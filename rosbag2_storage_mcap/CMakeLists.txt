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

include(FetchContent)
fetchcontent_declare(mcap
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG releases/cpp/v0.1.1
  GIT_SHALLOW 1
)
fetchcontent_makeavailable(mcap)

fetchcontent_declare(lz4
  GIT_REPOSITORY https://github.com/lz4/lz4.git
  GIT_TAG v1.9.3
  GIT_SHALLOW 1
)
fetchcontent_makeavailable(lz4)

fetchcontent_declare(zstd
  GIT_REPOSITORY https://github.com/facebook/zstd.git
  GIT_TAG v1.5.2
  GIT_SHALLOW 1
)
fetchcontent_makeavailable(zstd)

file(GLOB _zstd_sources
  ${zstd_SOURCE_DIR}/lib/common/*.c
  ${zstd_SOURCE_DIR}/lib/compress/*.c
  ${zstd_SOURCE_DIR}/lib/decompress/*.c
  ${zstd_SOURCE_DIR}/lib/decompress/*.S
)
add_library(
  ${PROJECT_NAME} SHARED
  src/mcap_storage.cpp
  src/message_definition_cache.cpp
  ${_zstd_sources}
  ${lz4_SOURCE_DIR}/lib/lz4.c
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME}
  SYSTEM PRIVATE
    ${mcap_SOURCE_DIR}/cpp/mcap/include
    ${lz4_SOURCE_DIR}/lib
    ${zstd_SOURCE_DIR}/lib
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
