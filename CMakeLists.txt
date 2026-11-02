cmake_minimum_required(VERSION 3.20)
project(multiobjective_search_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(MOSP_NATIVE "Compile Release builds with -march=native" ON)
option(MOSP_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(MOSP_BUILD_TESTS "Build the unit tests" ON)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
if(MOSP_NATIVE)
  string(APPEND CMAKE_CXX_FLAGS_RELEASE " -march=native")
endif()
if(MOSP_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_library(mosp
  src/dimacs.cpp
  src/graph.cpp
)
target_include_directories(mosp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MOSP_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(mosp_tests
    tests/test_dimacs.cpp
  )
  target_link_libraries(mosp_tests PRIVATE mosp GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(mosp_tests DISCOVERY_TIMEOUT 60)
endif()
