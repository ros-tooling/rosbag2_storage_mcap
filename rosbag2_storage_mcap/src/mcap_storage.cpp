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

#include <filesystem>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include "rcutils/logging_macros.h"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

namespace rosbag2_storage_plugins
{

/**
 * A storage implementation for the MCAP file format.
 */
class MCAPStorage
  : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  MCAPStorage();
  virtual ~MCAPStorage();

  /** BaseIOInterface **/
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    rosbag2_storage::storage_interfaces::IOFlag io_flag =
    rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

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
  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;
  void reset_filter() override;
  void seek(const rcutils_time_point_value_t & timestamp) override;

  /** ReadWriteInterface **/
  uint64_t get_minimum_split_file_size() const override;

  /** BaseWriteInterface **/
  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;
  void write(
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msg)
  override;
  void create_topic(const rosbag2_storage::TopicMetadata & topic) override;
  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override;

private:
  bool read_and_enqueue_message();

  std::optional<rosbag2_storage::storage_interfaces::IOFlag> opened_as_;
  std::string relative_path_;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_;

  rosbag2_storage::BagMetadata metadata_;
  std::unordered_map<std::string, rosbag2_storage::TopicInformation> topics_;
  rosbag2_storage::StorageFilter storage_filter_;
  std::unordered_set<std::string> filter_topics_;

  std::unique_ptr<std::ifstream> input_;
  std::unique_ptr<mcap::FileStreamReader> data_source_;
  std::unique_ptr<mcap::McapReader> mcap_reader_;
  std::unique_ptr<mcap::TypedRecordReader> typed_record_reader_;
};

MCAPStorage::MCAPStorage()
: mcap_reader_(std::make_unique<mcap::McapReader>())
{
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.message_count = 0;
}

MCAPStorage::~MCAPStorage()
{
  // A bag that has not been opened needs no finalizing.
  if (!opened_as_) {
    return;
  }

  if (opened_as_ == rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    mcap_reader_->close();
  }

  // In the case of READ_WRITE (write) - we need to write the final information to the file.
  if (opened_as_ == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    // TODO(mcap)
  }
}

/** BaseIOInterface **/
void MCAPStorage::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  switch (io_flag) {
    case rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE:
      // TODO(mcap)
      throw std::runtime_error("MCAPStorage does not support READ_WRITE mode");
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY:
      {
        relative_path_ = storage_options.uri;
        input_ = std::make_unique<std::ifstream>(relative_path_, std::ios::binary);
        data_source_ = std::make_unique<mcap::FileStreamReader>(*input_);
        typed_record_reader_ = std::make_unique<mcap::TypedRecordReader>(*data_source_, 8);
        auto status = mcap_reader_->open(*data_source_);
        if (!status.ok()) {
          throw std::runtime_error(status.message);
        }
        break;
      }
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND:
      // TODO(mcap)
      throw std::runtime_error("MCAPStorage does not support APPEND mode");
      break;
  }
  opened_as_ = io_flag;
  metadata_.relative_file_paths = {get_relative_file_path()};
}

/** BaseInfoInterface **/
rosbag2_storage::BagMetadata MCAPStorage::get_metadata()
{
  metadata_.version = 2;
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.bag_size = get_bagfile_size();
  metadata_.relative_file_paths = {get_relative_file_path()};
  std::unordered_map<mcap::SchemaId,
    rosbag2_storage::TopicInformation> topic_information_schema_map;
  std::unordered_map<mcap::ChannelId,
    rosbag2_storage::TopicInformation> topic_information_channel_map;
  typed_record_reader_->onSchema =
    [&topic_information_schema_map, this](const mcap::SchemaPtr schema_ptr) {
      rosbag2_storage::TopicInformation topic_info{};
      topic_info.topic_metadata.type = schema_ptr->name;
      topic_info.topic_metadata.serialization_format = get_storage_identifier();
      topic_information_schema_map.insert({schema_ptr->id, topic_info});
    };
  typed_record_reader_->onChannel =
    [&topic_information_schema_map,
      &topic_information_channel_map](const mcap::ChannelPtr channel_ptr) {
      auto topic_info = topic_information_schema_map[channel_ptr->schemaId];
      topic_info.topic_metadata.name = channel_ptr->topic;
      topic_information_channel_map.insert({channel_ptr->id, topic_info});
    };
  typed_record_reader_->onStatistics =
    [&topic_information_channel_map, this](const mcap::Statistics & statistics) {
      metadata_.message_count = statistics.messageCount;
      metadata_.duration = std::chrono::nanoseconds(
        statistics.messageEndTime - statistics.messageStartTime);
      metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::nanoseconds(statistics.messageStartTime));
      for (auto & topic_info_with_id : topic_information_channel_map) {
        if (statistics.channelMessageCounts.find(topic_info_with_id.first) !=
          statistics.channelMessageCounts.end())
        {
          topic_info_with_id.second.message_count = statistics.channelMessageCounts.at(
            topic_info_with_id.first);
        }
        metadata_.topics_with_message_count.push_back(topic_info_with_id.second);
      }
    };
  bool running = true;
  while (running) {
    running = typed_record_reader_->next();
    if (!typed_record_reader_->status().ok()) {
      throw std::runtime_error(typed_record_reader_->status().message);
    }
  }
  return metadata_;
}

std::string MCAPStorage::get_relative_file_path() const
{
  return relative_path_;
}

uint64_t MCAPStorage::get_bagfile_size() const
{
  return data_source_->size();
}

std::string MCAPStorage::get_storage_identifier() const
{
  return "mcap";
}

/** BaseReadInterface **/
bool MCAPStorage::read_and_enqueue_message()
{
  // The bag has not been opened.
  if (!opened_as_) {
    return false;
  }
  // Already have popped and queued the next message.
  if (next_ != nullptr) {
    return true;
  }

  auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  // TODO(mcap): fill msg

  // enqueue this message to be used
  next_ = msg;
  return true;
}

bool MCAPStorage::has_next()
{
  if (!opened_as_) {
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

std::shared_ptr<rosbag2_storage::SerializedBagMessage> MCAPStorage::read_next()
{
  if (!has_next()) {
    throw std::runtime_error{"No next message is available."};
  }
  // Importantly, clear next_ via move so that a next message can be read.
  return std::move(next_);
}

std::vector<rosbag2_storage::TopicMetadata> MCAPStorage::get_all_topics_and_types()
{
  auto metadata = get_metadata();
  std::vector<rosbag2_storage::TopicMetadata> out;
  for (const auto & topic : metadata.topics_with_message_count) {
    out.push_back(topic.topic_metadata);
  }
  return out;
}

/** ReadOnlyInterface **/
void MCAPStorage::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
  storage_filter_ = storage_filter;
  filter_topics_.clear();
  filter_topics_.insert(storage_filter.topics.begin(), storage_filter.topics.end());
}

void MCAPStorage::reset_filter()
{
  set_filter(rosbag2_storage::StorageFilter());
}

void MCAPStorage::seek(const rcutils_time_point_value_t & /* time_stamp */)
{
  // TODO(mcap)
}

/** ReadWriteInterface **/
uint64_t MCAPStorage::get_minimum_split_file_size() const
{
  // TODO(mcap)
  return 0;
}

/** BaseWriteInterface **/
void MCAPStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  // TODO(mcap) - also note that metadata update could be done differently
  // - maybe MCAP library tracks those values and we should just query them

  /// Update metadata
  // Increment individual topic message count
  topics_.at(msg->topic_name).message_count++;
  // Increment global message count
  metadata_.message_count++;
  // Determine bag duration. Note: this assumes in-order writes.
  const auto chrono_ts = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(msg->time_stamp));
  metadata_.duration = chrono_ts - metadata_.starting_time;
}

void MCAPStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs)
{
  for (const auto & msg : msgs) {
    write(msg);
  }
}

void MCAPStorage::create_topic(const rosbag2_storage::TopicMetadata & topic)
{
  // TODO(mcap)
  if (topics_.find(topic.name) == topics_.end()) {
    topics_[topic.name] = rosbag2_storage::TopicInformation{topic, 0};
  }
}

void MCAPStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  // TODO(mcap)
  topics_.erase(topic.name);
}


}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_plugins::MCAPStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
