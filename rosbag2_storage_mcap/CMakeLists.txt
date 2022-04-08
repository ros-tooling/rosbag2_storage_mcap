cmake_minimum_required(VERSION 3.14)
project(rosbag2_storage_mcap LANGUAGES C CXX ASM)

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
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(ros_environment REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ament_index_cpp REQUIRED)

file(GLOB ZSTD_SRCS
  vendor/zstd/lib/common/*.c
  vendor/zstd/lib/compress/*.c
  vendor/zstd/lib/decompress/*.c
  vendor/zstd/lib/decompress/*.S
)
add_library(
  ${PROJECT_NAME} SHARED
  src/mcap_storage.cpp
  src/message_definition_cache.cpp
  ${ZSTD_SRCS}
  vendor/lz4/lib/lz4.c
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME}
  SYSTEM PRIVATE
    ${PROJECT_SOURCE_DIR}/vendor/mcap/cpp/mcap/include
    ${PROJECT_SOURCE_DIR}/vendor/lz4/lib
    ${PROJECT_SOURCE_DIR}/vendor/zstd/lib
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROS_DISTRO=${ROS_DISTRO})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_STORAGE_MCAP_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rcutils
  ros_environment
  rosbag2_storage
)

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  file(GLOB _all_non_vendor_files *)
  list(REMOVE_ITEM _all_non_vendor_files ${PROJECT_SOURCE_DIR}/vendor)

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(${_all_non_vendor_files})

  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(src)

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(src)

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(src)

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(CMakeLists.txt)

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosbag2_storage rcutils)

ament_package()
