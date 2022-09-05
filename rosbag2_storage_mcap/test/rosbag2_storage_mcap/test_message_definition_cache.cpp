#include "gmock/gmock.h"
#include "rosbag2_storage_mcap/message_definition_cache.hpp"

using ::testing::UnorderedElementsAre;
using namespace rosbag2_storage_mcap::internal;

TEST(test_message_definition_cache, can_find_idl_includes) {
  const char sample[] = R"r(
#include <autoware_auto_control_msgs/msg/AckermannLateralCommand.idl>
#include "autoware_auto_control_msgs/msg/LongitudinalCommand.idl"
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
  std::set<std::string> dependencies =
    parse_dependencies(MessageDefinitionFormat::ROS2IDL, sample, "");
  ASSERT_THAT(dependencies,
              UnorderedElementsAre("autoware_auto_control_msgs/msg/AckermannLateralCommand",
                                   "autoware_auto_control_msgs/msg/LongitudinalCommand",
                                   "builtin_interfaces/msg/Time"));
}
