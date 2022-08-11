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
#include "rosbag2_storage/storage_options.hpp"
#include "std_msgs/msg/string.hpp"

#include <gmock/gmock.h>

using namespace ::testing;      // NOLINT
using namespace rcpputils::fs;  // NOLINT

TEST(TestMCAPStorage, can_write_and_read_basic_mcap_file) {
  const path tmp = create_temp_directory("test_mcap_bag", current_path());
  const path uri = tmp / "bag";
  const path expected_bag = uri / "bag_0.mcap";
  const int64_t timestamp_nanos = 100;  // arbitrary value
  rclcpp::Time time_stamp{timestamp_nanos};
  const std::string topic_name = "test_topic";
  const std::string message_data = "Test Message 1";
  const std::string storage_id = "mcap";

  {
    rosbag2_storage::StorageOptions options;
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
    rosbag2_storage::StorageOptions options;
    options.uri = expected_bag.string();
    options.storage_id = storage_id;

    rosbag2_cpp::Reader reader{};
    reader.open(options);
    EXPECT_TRUE(reader.has_next());
    auto msg = reader.read_next<std_msgs::msg::String>();
    EXPECT_EQ(msg.data, message_data);
  }
  // rcpputils::fs::remove_all(tmp);
}
