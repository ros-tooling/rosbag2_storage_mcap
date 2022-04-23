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


## Define vendor macro
macro(build_mcap_vendor)
  include(FetchContent)
  fetchcontent_declare(mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG 743b1f8e6ac072db0649fe26ddb5d04534716fd3 # releases/cpp/v0.1.1
  )
  fetchcontent_makeavailable(mcap)

  fetchcontent_declare(lz4
    GIT_REPOSITORY https://github.com/lz4/lz4.git
    GIT_TAG d44371841a2f1728a3f36839fd4b7e872d0927d3 # v1.9.3
  )
  fetchcontent_makeavailable(lz4)

  fetchcontent_declare(zstd
    GIT_REPOSITORY https://github.com/facebook/zstd.git
    GIT_TAG e47e674cd09583ff0503f0f6defd6d23d8b718d3 # v1.5.2
  )
  fetchcontent_makeavailable(zstd)

  file(GLOB _zstd_srcs
    ${zstd_SOURCE_DIR}/lib/common/*.c
    ${zstd_SOURCE_DIR}/lib/compress/*.c
    ${zstd_SOURCE_DIR}/lib/decompress/*.c
    ${zstd_SOURCE_DIR}/lib/decompress/*.S
  )
  add_library(
    mcap SHARED
    src/main.cpp
    ${_zstd_srcs}
    ${lz4_SOURCE_DIR}/lib/lz4.c
  )

  set(_mcap_include_dir ${mcap_SOURCE_DIR}/cpp/mcap/include)

  target_include_directories(mcap SYSTEM PRIVATE
    ${lz4_SOURCE_DIR}/lib
    ${zstd_SOURCE_DIR}/lib
  )
  target_include_directories(mcap SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${_mcap_include_dir}>"
    "$<INSTALL_INTERFACE:include>"
  )

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
