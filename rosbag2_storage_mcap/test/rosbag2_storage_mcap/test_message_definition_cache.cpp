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

#include "gmock/gmock.h"
#include "rosbag2_storage_mcap/message_definition_cache.hpp"

#include <set>
#include <string>

using rosbag2_storage_mcap::internal::Format;
using rosbag2_storage_mcap::internal::parse_dependencies;
using ::testing::UnorderedElementsAre;

TEST(test_message_definition_cache, can_find_idl_includes) {
  const char sample[] = R"r(
#include "autoware_auto_control_msgs/msg/LongitudinalCommand.idl"

#include <autoware_auto_control_msgs/msg/AckermannLateralCommand.idl>
// trailing spaces
#include "builtin_interfaces/msg/Time.idl"   

module autoware_auto_control_msgs {
  module msg {
    @verbatim (language="comment", text=
      " Lateral and longitudinal control message for Ackermann-style platforms")
    struct AckermannControlCommand {
      builtin_interfaces::msg::Time stamp;

      autoware_auto_control_msgs::msg::AckermannLateralCommand lateral;
      autoware_auto_control_msgs::msg::LongitudinalCommand longitudinal;
    };
  };
};
)r";
  std::set<std::string> dependencies = parse_dependencies(Format::IDL, sample, "");
  ASSERT_THAT(dependencies,
              UnorderedElementsAre("autoware_auto_control_msgs/msg/AckermannLateralCommand",
                                   "autoware_auto_control_msgs/msg/LongitudinalCommand",
                                   "builtin_interfaces/msg/Time"));
}
