cmake_minimum_required(VERSION 3.14)
project(etrobocon2026)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

option(ENABLE_COVERAGE "Enable test coverage reporting" ON)
if(ENABLE_COVERAGE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(RUNNING_MODULE_DIR ${PROJECT_SOURCE_DIR}/modules)
set(RUNNING_TEST_DIR ${PROJECT_SOURCE_DIR}/tests)
set(CAMERA_MODULE_DIR ${PROJECT_SOURCE_DIR}/camera_server/modules)
set(CAMERA_TEST_DIR ${PROJECT_SOURCE_DIR}/camera_server/tests)

# OpenSSL
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENSSL REQUIRED openssl)

enable_testing()
include(GoogleTest)