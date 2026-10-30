cmake_minimum_required(VERSION 3.14)
project(aero_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(aero_core
  src/geo.cpp
  src/aircraft.cpp
  src/sensor.cpp
  src/tracker.cpp
  src/anomaly.cpp
  src/event_log.cpp)
target_include_directories(aero_core PUBLIC include)
if(MSVC)
  target_compile_options(aero_core PRIVATE /W4)
else()
  target_compile_options(aero_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_executable(aero_sim src/main.cpp)
target_link_libraries(aero_sim PRIVATE aero_core)

# --- Tests (GoogleTest) ---
include(FetchContent)
FetchContent_Declare(googletest
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(aero_tests tests/test_geo.cpp tests/test_anomaly.cpp tests/test_aircraft.cpp)
target_link_libraries(aero_tests PRIVATE aero_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(aero_tests)
