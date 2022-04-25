// Copyright 2022, Amazon.com Inc or its Affiliates. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "message_definition_cache.hpp"
#include "rcutils/logging_macros.h"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include <mcap/mcap.hpp>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace rosbag2_storage_plugins {

using mcap::ByteOffset;
using time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
static const char FILE_EXTENSION[] = ".mcap";

/**
 * A storage implementation for the MCAP file format.
 */
class MCAPStorage : public rosbag2_storage::storage_interfaces::ReadWriteInterface {
public:
  MCAPStorage();
  virtual ~MCAPStorage();

  /** BaseIOInterface **/
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
  void open(const rosbag2_storage::StorageOptions& storage_options,
            rosbag2_storage::storage_interfaces::IOFlag io_flag =
              rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;
  void open(const std::string& uri, rosbag2_storage::storage_interfaces::IOFlag io_flag =
                                      rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE);
#else
  void open(const std::string& uri,
            rosbag2_storage::storage_interfaces::IOFlag io_flag =
              rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;
#endif

  /** BaseInfoInterface **/
  rosbag2_storage::BagMetadata get_metadata() override;
  std::string get_relative_file_path() const override;
  uint64_t get_bagfile_size() const override;
  std::string get_storage_identifier() const override;

  /** BaseReadInterface **/
  bool has_next() override;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;

  /** ReadOnlyInterface **/
  void set_filter(const rosbag2_storage::StorageFilter& storage_filter) override;
  void reset_filter() override;
#ifdef ROSBAG2_STORAGE_MCAP_OVERRIDE_SEEK_METHOD
  void seek(const rcutils_time_point_value_t& timestamp) override;
#else
  void seek(const rcutils_time_point_value_t& timestamp);
#endif

  /** ReadWriteInterface **/
  uint64_t get_minimum_split_file_size() const override;

  /** BaseWriteInterface **/
  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;
  void write(
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>& msg) override;
  void create_topic(const rosbag2_storage::TopicMetadata& topic) override;
  void remove_topic(const rosbag2_storage::TopicMetadata& topic) override;

private:
  bool read_and_enqueue_message();
  void ensure_summary_read();

  std::optional<rosbag2_storage::storage_interfaces::IOFlag> opened_as_;
  std::string relative_path_;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_;

  rosbag2_storage::BagMetadata metadata_;
  std::unordered_map<std::string, rosbag2_storage::TopicInformation> topics_;
  std::unordered_map<std::string, mcap::SchemaId> schema_ids;    // datatype -> schema_id
  std::unordered_map<std::string, mcap::ChannelId> channel_ids;  // topic -> channel_id
  rosbag2_storage::StorageFilter storage_filter_;
  std::unordered_set<std::string> filter_topics_;

  std::unique_ptr<std::ifstream> input_;
  std::unique_ptr<mcap::FileStreamReader> data_source_;
  std::unique_ptr<mcap::McapReader> mcap_reader_;
  std::unique_ptr<mcap::LinearMessageView> linear_view_;
  std::unique_ptr<mcap::LinearMessageView::Iterator> linear_iterator_;

  std::unique_ptr<mcap::McapWriter> mcap_writer_;
  rosbag2_storage_mcap::internal::MessageDefinitionCache msgdef_cache_;

  bool has_read_summary_ = false;
};

MCAPStorage::MCAPStorage() {
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.message_count = 0;
}

MCAPStorage::~MCAPStorage() {
  if (mcap_reader_) {
    mcap_reader_->close();
  }
  if (input_) {
    input_->close();
  }
  if (mcap_writer_) {
    mcap_writer_->close();
  }
}

/** BaseIOInterface **/
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
void MCAPStorage::open(const rosbag2_storage::StorageOptions& storage_options,
                       rosbag2_storage::storage_interfaces::IOFlag io_flag) {
  open(storage_options.uri, io_flag);
}
#endif

void MCAPStorage::open(const std::string& uri,
                       rosbag2_storage::storage_interfaces::IOFlag io_flag) {
  switch (io_flag) {
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY: {
      relative_path_ = uri;
      input_ = std::make_unique<std::ifstream>(relative_path_, std::ios::binary);
      data_source_ = std::make_unique<mcap::FileStreamReader>(*input_);
      mcap_reader_ = std::make_unique<mcap::McapReader>();
      auto status = mcap_reader_->open(*data_source_);
      if (!status.ok()) {
        throw std::runtime_error(status.message);
      }
      linear_view_ = std::make_unique<mcap::LinearMessageView>(mcap_reader_->readMessages());
      linear_iterator_ = std::make_unique<mcap::LinearMessageView::Iterator>(linear_view_->begin());
      break;
    }
    case rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE:
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND: {
      // APPEND does not seem to be used; treat it the same as READ_WRITE
      io_flag = rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE;
      relative_path_ = uri + FILE_EXTENSION;

      mcap_writer_ = std::make_unique<mcap::McapWriter>();
      mcap::McapWriterOptions options{"ros2"};
      auto status = mcap_writer_->open(relative_path_, options);
      if (!status.ok()) {
        throw std::runtime_error(status.message);
      }
      break;
    }
  }
  opened_as_ = io_flag;
  metadata_.relative_file_paths = {get_relative_file_path()};
}

/** BaseInfoInterface **/
rosbag2_storage::BagMetadata MCAPStorage::get_metadata() {
  ensure_summary_read();

  metadata_.version = 2;
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.bag_size = get_bagfile_size();
  metadata_.relative_file_paths = {get_relative_file_path()};

  // Fill out summary metadata from the Statistics record
  const mcap::Statistics& stats = mcap_reader_->statistics().value();
  metadata_.message_count = stats.messageCount;
  metadata_.duration = std::chrono::nanoseconds(stats.messageEndTime - stats.messageStartTime);
  metadata_.starting_time = time_point(std::chrono::nanoseconds(stats.messageStartTime));

  // Build a list of topic information along with per-topic message counts
  metadata_.topics_with_message_count.clear();
  for (const auto& [channel_id, channel_ptr] : mcap_reader_->channels()) {
    const mcap::Channel& channel = *channel_ptr;

    // Look up the Schema for this topic
    const auto schema_ptr = mcap_reader_->schema(channel.schemaId);
    if (!schema_ptr) {
      throw std::runtime_error("Could not find schema for topic " + channel.topic);
    }

    // Create a TopicInformation for this topic
    rosbag2_storage::TopicInformation topic_info{};
    topic_info.topic_metadata.name = channel.topic;
    topic_info.topic_metadata.serialization_format = channel.messageEncoding;
    topic_info.topic_metadata.type = schema_ptr->name;

    // Look up the offered_qos_profiles metadata entry
    const auto metadata_it = channel.metadata.find("offered_qos_profiles");
    if (metadata_it != channel.metadata.end()) {
      topic_info.topic_metadata.offered_qos_profiles = metadata_it->second;
    }

    // Look up the message count for this Channel
    const auto message_count_it = stats.channelMessageCounts.find(channel_id);
    if (message_count_it != stats.channelMessageCounts.end()) {
      topic_info.message_count = message_count_it->second;
    } else {
      topic_info.message_count = 0;
    }

    metadata_.topics_with_message_count.push_back(topic_info);
  }

  return metadata_;
}

std::string MCAPStorage::get_relative_file_path() const {
  return relative_path_;
}

uint64_t MCAPStorage::get_bagfile_size() const {
  if (opened_as_ == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    return data_source_ ? data_source_->size() : 0;
  } else {
    if (!mcap_writer_) {
      return 0;
    }
    const auto* data_sink = mcap_writer_->dataSink();
    return data_sink ? data_sink->size() : 0;
  }
}

std::string MCAPStorage::get_storage_identifier() const {
  return "mcap";
}

/** BaseReadInterface **/
bool MCAPStorage::read_and_enqueue_message() {
  // The recording has not been opened.
  if (!linear_iterator_) {
    return false;
  }
  // Already have popped and queued the next message.
  if (next_ != nullptr) {
    return true;
  }

  auto& it = *linear_iterator_;

  // At the end of the recording
  if (it == linear_view_->end()) {
    return false;
  }

  const auto& messageView = *it;
  auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  msg->time_stamp = rcutils_time_point_value_t(messageView.message.logTime);
  msg->topic_name = messageView.channel->topic;
  msg->serialized_data = rosbag2_storage::make_serialized_message(messageView.message.data,
                                                                  messageView.message.dataSize);

  // enqueue this message to be used
  next_ = msg;

  ++it;
  return true;
}

void MCAPStorage::ensure_summary_read() {
  if (!has_read_summary_) {
    const auto status = mcap_reader_->readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!status.ok()) {
      throw std::runtime_error(status.message);
    }
    has_read_summary_ = true;
  }
}

bool MCAPStorage::has_next() {
  if (!linear_iterator_) {
    return false;
  }
  // Have already verified next message and enqueued it for use.
  if (next_) {
    return true;
  }

  // Continue reading messages until one matches the filter, or there are none left
  while (true) {
    if (!read_and_enqueue_message()) {
      return false;
    }
    if (filter_topics_.empty() || filter_topics_.count(next_->topic_name)) {
      break;
    }
    // Next message did not pass filter - throw it away and continue
    next_.reset();
  }
  return true;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> MCAPStorage::read_next() {
  if (!has_next()) {
    throw std::runtime_error{"No next message is available."};
  }
  // Importantly, clear next_ via move so that a next message can be read.
  return std::move(next_);
}

std::vector<rosbag2_storage::TopicMetadata> MCAPStorage::get_all_topics_and_types() {
  auto metadata = get_metadata();
  std::vector<rosbag2_storage::TopicMetadata> out;
  for (const auto& topic : metadata.topics_with_message_count) {
    out.push_back(topic.topic_metadata);
  }
  return out;
}

/** ReadOnlyInterface **/
void MCAPStorage::set_filter(const rosbag2_storage::StorageFilter& storage_filter) {
  storage_filter_ = storage_filter;
  filter_topics_.clear();
  filter_topics_.insert(storage_filter.topics.begin(), storage_filter.topics.end());
}

void MCAPStorage::reset_filter() {
  set_filter(rosbag2_storage::StorageFilter());
}

void MCAPStorage::seek(const rcutils_time_point_value_t& time_stamp) {
  ensure_summary_read();

  auto start_time = mcap::Timestamp(time_stamp);
  linear_view_ = std::make_unique<mcap::LinearMessageView>(mcap_reader_->readMessages(start_time));
  linear_iterator_ = std::make_unique<mcap::LinearMessageView::Iterator>(linear_view_->begin());
}

/** ReadWriteInterface **/
uint64_t MCAPStorage::get_minimum_split_file_size() const {
  return 1024;
}

/** BaseWriteInterface **/
void MCAPStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) {
  const auto topic_it = topics_.find(msg->topic_name);
  if (topic_it == topics_.end()) {
    throw std::runtime_error{"Unknown message topic \"" + msg->topic_name + "\""};
  }
  const auto& topic_info = topic_it->second;

  // Get or create a Schema reference
  mcap::SchemaId schema_id;
  const auto& datatype = topic_info.topic_metadata.type;
  const auto schema_it = schema_ids.find(datatype);
  if (schema_it == schema_ids.end()) {
    std::string full_text = msgdef_cache_.get_full_text(datatype);
    mcap::Schema schema;
    schema.name = datatype;
    schema.encoding = "ros2msg";
    schema.data.assign(reinterpret_cast<const std::byte*>(full_text.data()),
                       reinterpret_cast<const std::byte*>(full_text.data() + full_text.size()));
    mcap_writer_->addSchema(schema);
    schema_ids.emplace(datatype, schema.id);
    schema_id = schema.id;
  } else {
    schema_id = schema_it->second;
  }

  // Get or create a Channel reference
  mcap::ChannelId channel_id;
  const auto channel_it = channel_ids.find(msg->topic_name);
  if (channel_it == channel_ids.end()) {
    mcap::Channel channel;
    channel.topic = msg->topic_name;
    channel.messageEncoding = topic_info.topic_metadata.serialization_format;
    channel.schemaId = schema_id;
    channel.metadata.emplace("offered_qos_profiles",
                             topic_info.topic_metadata.offered_qos_profiles);
    mcap_writer_->addChannel(channel);
    channel_ids.emplace(msg->topic_name, channel.id);
    channel_id = channel.id;
  } else {
    channel_id = channel_it->second;
  }

  mcap::Message mcapMsg;
  mcapMsg.channelId = channel_id;
  mcapMsg.sequence = 0;
  mcapMsg.logTime = mcap::Timestamp(std::chrono::nanoseconds(msg->time_stamp).count());
  mcapMsg.publishTime = mcapMsg.logTime;
  mcapMsg.dataSize = msg->serialized_data->buffer_length;
  mcapMsg.data = reinterpret_cast<const std::byte*>(msg->serialized_data->buffer);
  const auto status = mcap_writer_->write(mcapMsg);
  if (!status.ok()) {
    throw std::runtime_error{std::string{"Failed to write "} +
                             std::to_string(msg->serialized_data->buffer_length) +
                             " byte message to MCAP file: " + status.message};
  }

  /// Update metadata
  // Increment individual topic message count
  topic_it->second.message_count++;
  // Increment global message count
  metadata_.message_count++;
  // Determine recording duration
  const auto message_time = time_point(std::chrono::nanoseconds(msg->time_stamp));
  metadata_.duration = std::max(metadata_.duration, message_time - metadata_.starting_time);
}

void MCAPStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>>& msgs) {
  for (const auto& msg : msgs) {
    write(msg);
  }
}

void MCAPStorage::create_topic(const rosbag2_storage::TopicMetadata& topic) {
  if (topics_.find(topic.name) == topics_.end()) {
    topics_.emplace(topic.name, rosbag2_storage::TopicInformation{topic, 0});
  }
}

void MCAPStorage::remove_topic(const rosbag2_storage::TopicMetadata& topic) {
  topics_.erase(topic.name);
}

}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::MCAPStorage,
                       rosbag2_storage::storage_interfaces::ReadWriteInterface)
