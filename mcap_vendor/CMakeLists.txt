cmake_minimum_required(VERSION 3.14)
project(mcap_vendor)

find_package(ament_cmake REQUIRED)

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
  GIT_TAG 44355df51a2f77b764b42bb9b3b0985200a168ad
)
fetchcontent_makeavailable(mcap)

set(MCAP_ROOT ${CMAKE_CURRENT_BINARY_DIR}/_deps/mcap-src/cpp)
set(MCAP_INCLUDE_DIRS ${MCAP_ROOT}/mcap/include)

conan_cmake_install(
  PATH_OR_REFERENCE ${MCAP_ROOT}/mcap
  INSTALL_FOLDER ${MCAP_ROOT}/mcap/build/Release
  BUILD missing
  SETTINGS compiler.cppstd=17
  SETTINGS build_type=Release
)

include(${MCAP_ROOT}/mcap/build/Release/conanbuildinfo.cmake)
conan_basic_setup()

add_library(mcap INTERFACE)
target_link_libraries(mcap INTERFACE
  ${CONAN_LIBS}
)
target_include_directories(mcap SYSTEM INTERFACE
  "$<BUILD_INTERFACE:${MCAP_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:include>"
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

ament_export_include_directories(include)

## Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_package()
