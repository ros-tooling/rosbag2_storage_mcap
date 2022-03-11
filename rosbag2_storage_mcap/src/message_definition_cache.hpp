#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rosbag2_storage_mcap::internal {

struct MessageSpec {
  MessageSpec(std::string text, const std::string& package_context);
  const std::set<std::string> dependencies;
  const std::string text;
};

class MessageDefinitionCache final {
public:
  MessageDefinitionCache();

  /**
   * Concatenate the message definition with its dependencies into a self-contained schema.
   * Uses a format similar to ROS 1's gendeps:
   * https://github.com/ros/ros/blob/93d8da32091b8b43702eab5d3202f4511dfeb7dc/core/roslib/src/roslib/gentools.py#L239
   */
  std::string get_full_text(const std::string& datatype);

private:
  /** Populate msg_files_by_package with the list of known message files per package */
  void load_message_packages();

  /**
   * Load and parse the message file referenced by the given datatype, or return it from
   * msg_specs_by_datatype
   */
  const MessageSpec& load_message_spec(const std::string& datatype);

  std::unordered_map<std::string, std::unordered_set<std::string>> msg_files_by_package_;
  std::unordered_map<std::string, MessageSpec> msg_specs_by_datatype_;
};

}  // namespace rosbag2_storage_mcap::internal
