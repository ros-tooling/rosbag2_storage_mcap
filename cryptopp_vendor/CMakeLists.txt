cmake_minimum_required(VERSION 3.5)
project(cryptopp_vendor)

find_package(ament_cmake REQUIRED)

option(FORCE_BUILD_VENDOR_PKG
  "Build cryptopp from source, even if system-installed package is available"
  OFF)

if(NOT FORCE_BUILD_VENDOR_PKG)
  list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
  find_package(CryptoPP REQUIRED)
endif()

if(NOT CryptoPP_FOUND OR "${CryptoPP_VERSION}" VERSION_LESS 8.6.0)
  set(extra_cmake_args)

  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

  set(CRYPTOPP_TAG CRYPTOPP_8_6_0)

  file(DOWNLOAD
    https://raw.githubusercontent.com/noloader/cryptopp-cmake/${CRYPTOPP_TAG}/CMakeLists.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/cryptopp_cmakelists.txt
    SHOW_PROGRESS
    TIMEOUT 60s
  )

  file(DOWNLOAD
    https://raw.githubusercontent.com/noloader/cryptopp-cmake/${CRYPTOPP_TAG}/cryptopp-config.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/cryptopp-config.cmake
    SHOW_PROGRESS
    TIMEOUT 60s
  )

  include(ExternalProject)
  ExternalProject_Add(cryptopp-8.6.0
    PREFIX crypto-8.6.0
    GIT_REPOSITORY https://github.com/weidai11/cryptopp.git
    GIT_TAG ${CRYPTOPP_TAG}
    UPDATE_COMMAND ""
    TIMEOUT 60
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/cryptopp_install
      ${extra_cmake_args}
    INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/cryptopp_install"
    PATCH_COMMAND
    ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/cryptopp_cmakelists.txt" <SOURCE_DIR>/CMakeLists.txt &&
    ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/cryptopp-config.cmake" <SOURCE_DIR>/cryptopp-config.cmake
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/cryptopp_install/
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS)
else ()
  message(STATUS "Found CryptoPP. Using CryptoPP from system.")
endif ()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "cryptopp_vendor-extras.cmake"
)
