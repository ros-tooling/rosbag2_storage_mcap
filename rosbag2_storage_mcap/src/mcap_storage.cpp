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
};

MCAPStorage::MCAPStorage()
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

  // A bag that is open for READ_ONLY needs no finalizing.
  // In the case of READ_WRITE (write) - we need to write the final information to the file.
  if (opened_as_ == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    // TODO(mcap)
  }
}

/** BaseIOInterface **/
void MCAPStorage::open(
  const rosbag2_storage::StorageOptions & /* storage_options */,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  switch (io_flag) {
    case rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE:
      // TODO(mcap)
      throw std::runtime_error("MCAPStorage does not support READ_WRITE mode");
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY:
      // TODO(mcap)
      throw std::runtime_error("MCAPStorage does not support READ_ONLY mode");
      break;
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
  // TODO(mcap) this implementation is just a sample, maybe this should be different
  if (opened_as_ == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    metadata_.topics_with_message_count.clear();
    for (const auto & kv : topics_) {
      metadata_.topics_with_message_count.push_back(kv.second);
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
  // TODO(mcap) is there a different way this should be calculated?
  return std::filesystem::file_size(relative_path_);
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
