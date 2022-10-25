cmake_minimum_required(VERSION 3.5)
project(mcap_vendor LANGUAGES C CXX ASM)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

## Dependencies
find_package(ament_cmake REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)


## Define vendor macro
macro(build_mcap_vendor)
  include(FetchContent)
  fetchcontent_declare(mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG 0aec14adccb1b39de835251cf0902a77ed41dcad # unreleased
  )
  fetchcontent_makeavailable(mcap)

  fetchcontent_declare(lz4
    GIT_REPOSITORY https://github.com/lz4/lz4.git
    GIT_TAG d44371841a2f1728a3f36839fd4b7e872d0927d3 # v1.9.3
  )
  fetchcontent_makeavailable(lz4)

  file(GLOB _lz4_srcs
    ${lz4_SOURCE_DIR}/lib/*.c)

  add_library(
    mcap SHARED
    src/main.cpp
    ${_lz4_srcs}
  )

  set(_mcap_include_dir ${mcap_SOURCE_DIR}/cpp/mcap/include)

  target_include_directories(mcap SYSTEM PRIVATE
    ${lz4_SOURCE_DIR}/lib
  )
  target_include_directories(mcap SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${_mcap_include_dir}>"
    "$<INSTALL_INTERFACE:include>"
  )
  ament_target_dependencies(mcap zstd)

  install(
    DIRECTORY
      ${_mcap_include_dir}/mcap
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/include
  )

  install(TARGETS mcap EXPORT export_mcap)
endmacro()

## Call vendor macro
build_mcap_vendor()

ament_export_targets(export_mcap HAS_LIBRARY_TARGET)

ament_export_dependencies(zstd_vendor zstd)

## Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_package()
