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
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(ros_environment REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
  message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
  file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/v0.16.1/conan.cmake"
                "${CMAKE_BINARY_DIR}/conan.cmake"
                EXPECTED_HASH SHA256=396e16d0f5eabdc6a14afddbcfff62a54a7ee75c6da23f32f7a31bc85db23484
                TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

include(FetchContent)
fetchcontent_declare(mcap
  GIT_REPOSITORY https://github.com/foxglove/mcap.git
  GIT_TAG 0851e316cf973c26a64773c56540efbbaf06725a
)
fetchcontent_makeavailable(mcap)

set(MCAP_ROOT ${CMAKE_CURRENT_BINARY_DIR}/_deps/mcap-src/cpp)
set(MCAP_INCLUDE_DIRS ${MCAP_ROOT}/mcap/include)

conan_cmake_autodetect(settings)

conan_cmake_install(
  PATH_OR_REFERENCE ${MCAP_ROOT}/mcap
  INSTALL_FOLDER ${MCAP_ROOT}/mcap/build/Release
  BUILD missing
  SETTINGS compiler=gcc
  SETTINGS compiler.version=11
  SETTINGS compiler.libcxx=libstdc++
  SETTINGS ${settings}
)

include(${MCAP_ROOT}/mcap/build/Release/conanbuildinfo.cmake)
conan_basic_setup()

add_library(
  ${PROJECT_NAME} SHARED
  src/mcap_storage.cpp
  src/message_definition_cache.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    $<BUILD_INTERFACE:${MCAP_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${CONAN_LIBS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROS_DISTRO=${ROS_DISTRO})

install(
  DIRECTORY
    ${MCAP_INCLUDE_DIRS}/mcap
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/include
)

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
