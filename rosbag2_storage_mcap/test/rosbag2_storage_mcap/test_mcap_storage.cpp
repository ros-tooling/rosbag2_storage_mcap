// Copyright 2022, Foxglove Technologies. All rights reserved.
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

#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#ifdef ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
#include "rosbag2_storage/storage_options.hpp"
using StorageOptions = rosbag2_storage::StorageOptions;
#else
#include "rosbag2_cpp/storage_options.hpp"
using StorageOptions = rosbag2_cpp::StorageOptions;
#endif
#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "std_msgs/msg/string.hpp"

#include <gmock/gmock.h>

#include <string>

using namespace ::testing;  // NOLINT
using TemporaryDirectoryFixture = rosbag2_test_common::TemporaryDirectoryFixture;

TEST_F(TemporaryDirectoryFixture, can_write_and_read_basic_mcap_file) {
  const auto uri = rcpputils::fs::path(temporary_dir_path_) / "bag";
  const auto expected_bag = uri / "bag_0.mcap";
  const int64_t timestamp_nanos = 100;  // arbitrary value
  rclcpp::Time time_stamp{timestamp_nanos};
  const std::string topic_name = "test_topic";
  const std::string message_data = "Test Message 1";
  const std::string storage_id = "mcap";
  {
    StorageOptions options;
    options.uri = uri.string();
    options.storage_id = storage_id;

    std_msgs::msg::String msg;
    msg.data = message_data;

    rosbag2_cpp::Writer writer{};
    writer.open(options);
    writer.write(msg, topic_name, time_stamp);

    EXPECT_TRUE(expected_bag.is_regular_file());
  }
  {
    StorageOptions options;
    options.uri = expected_bag.string();
    options.storage_id = storage_id;

    rosbag2_cpp::Reader reader{};
    reader.open(options);
    EXPECT_TRUE(reader.has_next());
    auto msg = reader.read_next<std_msgs::msg::String>();
    EXPECT_EQ(msg.data, message_data);
  }
}
