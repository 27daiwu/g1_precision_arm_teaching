cmake_minimum_required(VERSION 3.18)
project(g1_precision_arm_teaching LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(unitree_sdk2 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Threads REQUIRED)

add_library(g1_arm_core STATIC
  src/dds.cpp
  src/arm_sdk_controller.cpp
  src/waist_hold.cpp
  src/trajectory.cpp
  src/continuous_trajectory.cpp
  src/trajectory_processor.cpp
  src/trajectory_retimer.cpp
  src/feedback_replay_controller.cpp
  src/recorder.cpp
  src/logger.cpp
  src/stability_gate.cpp
)
target_include_directories(g1_arm_core PUBLIC include)
target_link_libraries(g1_arm_core PUBLIC unitree_sdk2 JsonCpp::JsonCpp Threads::Threads)
target_compile_options(g1_arm_core PRIVATE -Wall -Wextra -Wpedantic)

foreach(app record replay)
  add_executable(g1_${app} apps/${app}.cpp)
  target_link_libraries(g1_${app} PRIVATE g1_arm_core)
  target_compile_options(g1_${app} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

enable_testing()
add_executable(g1_core_test tests/core_test.cpp)
target_link_libraries(g1_core_test PRIVATE g1_arm_core)
add_test(NAME g1_core_offline COMMAND g1_core_test)
add_executable(g1_mapping_test tests/mapping_test.cpp)
target_link_libraries(g1_mapping_test PRIVATE g1_arm_core)
add_test(NAME g1_mapping_offline COMMAND g1_mapping_test)
add_test(NAME record_help COMMAND g1_record --help)
add_test(NAME replay_help COMMAND g1_replay --help)
