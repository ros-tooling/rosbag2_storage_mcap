cmake_minimum_required(VERSION 3.5)
project(lz4_cmake_module)

find_package(ament_cmake REQUIRED)

if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/Modules/FindLZ4.cmake")
  file(DOWNLOAD
    https://raw.githubusercontent.com/Kitware/VTK/v9.1.0/CMake/FindLZ4.cmake
    ${CMAKE_SOURCE_DIR}/cmake/Modules/FindLZ4.cmake
    SHOW_PROGRESS
    TIMEOUT 60s
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  # Find module is intentionally mixed case, so tell linter to ignore that
  ament_lint_cmake("--filters=-convention/filename,-package/stdargs,-whitespace/extra")
  ament_copyright("--exclude cmake/Modules/FindLZ4.cmake")
  ament_xmllint()
endif()

ament_package(
  CONFIG_EXTRAS "lz4_cmake_module-extras.cmake"
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})
