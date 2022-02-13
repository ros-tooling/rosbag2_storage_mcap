cmake_minimum_required(VERSION 3.5)
project(mcap_vendor)

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
find_package(cryptopp_vendor REQUIRED)
find_package(CryptoPP REQUIRED)
find_package(fmt REQUIRED)
find_package(lz4_cmake_module REQUIRED)
find_package(LZ4 REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)


## Define vendor macro
macro(build_mcap_vendor)
  include(FetchContent)
  fetchcontent_declare(mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG dbe02410ba4899f055ea51599cee752c14bddd6d
  )
  fetchcontent_makeavailable(mcap)

  add_library(mcap INTERFACE)
  target_link_libraries(mcap INTERFACE
    CryptoPP::CryptoPP
    fmt::fmt
    LZ4::LZ4
    zstd::zstd
  )

  set(MCAP_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/_deps/mcap-src/cpp/mcap/include)

  target_include_directories(mcap SYSTEM INTERFACE
    "$<BUILD_INTERFACE:${MCAP_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:include>")

  set(MCAPDUMP_SRC ${CMAKE_CURRENT_BINARY_DIR}/_deps/mcap-src/cpp/examples/mcapdump.cpp)

  add_executable(mcapdump
    ${MCAPDUMP_SRC}
  )

  target_link_libraries(mcapdump
    mcap
  )

  install(
    DIRECTORY
      ${MCAP_INCLUDE_DIRS}/mcap
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/include
  )

  install(TARGETS mcap EXPORT export_mcap)

  install(EXPORT export_mcap
    FILE mcap-config.cmake
    DESTINATION share/cmake/mcap/
    EXPORT_LINK_INTERFACE_LIBRARIES)

  install(
    TARGETS mcapdump
    DESTINATION bin
  )
endmacro()

## Call vendor macro
build_mcap_vendor()

ament_export_include_directories(include)

ament_export_dependencies(cryptopp_vendor)
ament_export_dependencies(CryptoPP)
ament_export_dependencies(lz4_cmake_module)
ament_export_dependencies(LZ4)
ament_export_dependencies(fmt)
ament_export_dependencies(zstd_vendor)
ament_export_dependencies(zstd)

## Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_package()
