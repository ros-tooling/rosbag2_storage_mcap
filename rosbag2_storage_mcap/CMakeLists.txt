cmake_minimum_required(VERSION 3.8)
project(rosbag2_storage_mcap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rcutils REQUIRED)

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
  GIT_TAG c7ac48c02cad025f0694e584b3afd01702914ee5
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

add_library(${PROJECT_NAME} SHARED src/mcap_storage.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    $<BUILD_INTERFACE:${MCAP_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${CONAN_LIBS}
)

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
