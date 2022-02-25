cmake_minimum_required(VERSION 3.8)
project(rosbag2_storage_mcap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rcutils REQUIRED)

add_library(${PROJECT_NAME} SHARED src/mcap_storage.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_STORAGE_MCAP_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rcutils
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosbag2_storage rcutils)

ament_package()
