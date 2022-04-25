cmake_minimum_required(VERSION 3.14)
project(rosbag2_storage_mcap)

# Set Release build if no build type was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Build type for the build. Possible values are: Debug, Release, RelWithDebInfo, MinSizeRel"
      FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Enable additional warnings and warnings as errors
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get the ROS_DISTRO environment variable
set(ROS_DISTRO $ENV{ROS_DISTRO})

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mcap_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/mcap_storage.cpp
  src/message_definition_cache.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

target_link_libraries(${PROJECT_NAME}
  mcap_vendor::mcap
  pluginlib::pluginlib
  rcutils::rcutils
  rosbag2_storage::rosbag2_storage
)

if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL 0.10.0)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    ROSBAG2_STORAGE_MCAP_OVERRIDE_SEEK_METHOD
  )
endif()
if(${rosbag2_storage_VERSION} VERSION_GREATER_EQUAL 0.4.0)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    ROSBAG2_STORAGE_MCAP_HAS_STORAGE_OPTIONS
  )
endif()

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_STORAGE_MCAP_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosbag2_storage rcutils)

ament_package()
