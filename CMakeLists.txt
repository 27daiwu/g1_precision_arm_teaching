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
  src/calligraphy_trajectory.cpp
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

add_executable(g1_calligraphy_record apps/record.cpp)
target_link_libraries(g1_calligraphy_record PRIVATE g1_arm_core)
target_compile_options(g1_calligraphy_record PRIVATE -Wall -Wextra -Wpedantic)
add_executable(g1_replay apps/replay.cpp)
target_link_libraries(g1_replay PRIVATE g1_arm_core)
target_compile_options(g1_replay PRIVATE -Wall -Wextra -Wpedantic)
add_executable(g1_calligraphy_replay apps/calligraphy_replay.cpp)
target_link_libraries(g1_calligraphy_replay PRIVATE g1_arm_core)
target_compile_options(g1_calligraphy_replay PRIVATE -Wall -Wextra -Wpedantic)
add_executable(g1_calligraphy apps/g1_calligraphy.cpp)
target_link_libraries(g1_calligraphy PRIVATE JsonCpp::JsonCpp)
target_compile_options(g1_calligraphy PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(g1_core_test tests/core_test.cpp)
target_link_libraries(g1_core_test PRIVATE g1_arm_core)
add_test(NAME g1_core_offline COMMAND g1_core_test)
add_executable(g1_mapping_test tests/mapping_test.cpp)
target_link_libraries(g1_mapping_test PRIVATE g1_arm_core)
add_test(NAME g1_mapping_offline COMMAND g1_mapping_test)
add_executable(g1_calligraphy_replay_mock_test tests/calligraphy_replay_mock_test.cpp)
target_link_libraries(g1_calligraphy_replay_mock_test PRIVATE g1_arm_core)
add_test(NAME calligraphy_replay_mock COMMAND g1_calligraphy_replay_mock_test)
add_test(NAME calligraphy_record_help COMMAND g1_calligraphy_record --help)
add_test(NAME calligraphy_bias_limit
  COMMAND g1_calligraphy_record test --interface offline --waist-pitch-extra-bias 0.051)
set_tests_properties(calligraphy_bias_limit PROPERTIES WILL_FAIL TRUE)
add_test(NAME replay_help COMMAND g1_replay --help)
add_test(NAME replay_bias_limit
  COMMAND g1_replay test --preview --waist-pitch-extra-bias 0.051)
set_tests_properties(replay_bias_limit PROPERTIES WILL_FAIL TRUE)
add_test(NAME calligraphy_replay_help COMMAND g1_calligraphy_replay --help)
add_test(NAME unified_calligraphy_help COMMAND g1_calligraphy --help)
