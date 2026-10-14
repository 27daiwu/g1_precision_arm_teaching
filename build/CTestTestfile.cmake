# CMake generated Testfile for 
# Source directory: /home/hebe/g1_precision_arm_teaching
# Build directory: /home/hebe/g1_precision_arm_teaching/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(g1_core_offline "/home/hebe/g1_precision_arm_teaching/build/g1_core_test")
set_tests_properties(g1_core_offline PROPERTIES  _BACKTRACE_TRIPLES "/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;32;add_test;/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;0;")
add_test(g1_mapping_offline "/home/hebe/g1_precision_arm_teaching/build/g1_mapping_test")
set_tests_properties(g1_mapping_offline PROPERTIES  _BACKTRACE_TRIPLES "/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;35;add_test;/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;0;")
add_test(record_help "/home/hebe/g1_precision_arm_teaching/build/g1_record" "--help")
set_tests_properties(record_help PROPERTIES  _BACKTRACE_TRIPLES "/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;36;add_test;/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;0;")
add_test(replay_help "/home/hebe/g1_precision_arm_teaching/build/g1_replay" "--help")
set_tests_properties(replay_help PROPERTIES  _BACKTRACE_TRIPLES "/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;37;add_test;/home/hebe/g1_precision_arm_teaching/CMakeLists.txt;0;")
