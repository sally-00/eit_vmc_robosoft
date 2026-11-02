cmake_minimum_required(VERSION 3.16)
project(franka_eit_vmc LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(Eigen3 REQUIRED)
add_library(eit_vmc_core src/core.cpp src/joystick_mechanism.cpp)
set_target_properties(eit_vmc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(eit_vmc_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(eit_vmc_core PUBLIC Eigen3::Eigen)
option(FRANKA_EIT_CORE_ONLY "Build robot-independent control mathematics and tests" OFF)
if(FRANKA_EIT_CORE_ONLY)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core eit_vmc_core)
  add_test(NAME core COMMAND test_core)
  add_executable(test_ball_vmc test/test_ball_vmc.cpp)
  target_link_libraries(test_ball_vmc eit_vmc_core)
  add_test(NAME ball_vmc COMMAND test_ball_vmc ${CMAKE_CURRENT_SOURCE_DIR}/test/ball_simulation_normal.tsv)
  return()
endif()
find_package(ament_cmake REQUIRED)
find_package(Franka REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/EitContact.msg" "msg/ExperimentState.msg" "msg/JoystickState.msg" DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(message_support ${PROJECT_NAME} rosidl_typesupport_cpp)
add_library(eit_vmc_runner src/runner.cpp)
add_custom_target(recording_build_snapshot
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_build_snapshot.py
    --source ${CMAKE_CURRENT_SOURCE_DIR} --output ${CMAKE_CURRENT_BINARY_DIR}/provenance
    --build-dir ${CMAKE_CURRENT_BINARY_DIR}
    --compiler "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}"
    --dependency-versions "Franka=${Franka_VERSION},Eigen3=${Eigen3_VERSION},rclcpp=${rclcpp_VERSION}"
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/provenance/recording_build.hpp
  VERBATIM)
add_dependencies(eit_vmc_runner recording_build_snapshot)
target_include_directories(eit_vmc_runner PUBLIC include PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/provenance)
target_link_libraries(eit_vmc_runner PUBLIC eit_vmc_core Franka::Franka "${message_support}")
ament_target_dependencies(eit_vmc_runner PUBLIC rclcpp rcl_interfaces sensor_msgs std_msgs std_srvs)
foreach(executable joystick_controller ball_roll_controller move_ready)
  add_executable(${executable} src/${executable}.cpp)
  target_link_libraries(${executable} eit_vmc_runner)
  install(TARGETS ${executable} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(TARGETS eit_vmc_core eit_vmc_runner ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})
# Keep deployment tools adjacent to their executable wrappers; no source-workspace paths.
install(DIRECTORY python/franka_eit_tools DESTINATION lib/${PROJECT_NAME}
  COMPONENT joystick_recorder PATTERN __pycache__ EXCLUDE)
# The installed block runner must not depend on locating the source checkout.
# Keep both ESP32 recorder files together because the shell wrapper uses its own directory.
install(PROGRAMS
  ../joystick_hardware/run_joystick.sh
  ../joystick_hardware/plot_joystick.py
  DESTINATION lib/${PROJECT_NAME}/joystick_hardware
  COMPONENT joystick_recorder)
install(PROGRAMS scripts/run_block.py scripts/joystick_feedback_node.py
  DESTINATION lib/${PROJECT_NAME}
  COMPONENT joystick_recorder)
install(PROGRAMS scripts/eit_contact_node.py scripts/replay_ball.py scripts/replay_joystick.py
  scripts/eit_pad_viewer.py scripts/configure_mount.py scripts/capture_posture.py scripts/experiment.py scripts/plot_joystick_results.py scripts/inspect_model.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE VALIDATION.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_core test/test_core.cpp)
  target_link_libraries(test_core eit_vmc_core)
  add_test(NAME core COMMAND test_core)
  add_executable(test_ball_vmc test/test_ball_vmc.cpp)
  target_link_libraries(test_ball_vmc eit_vmc_core)
  add_test(NAME ball_vmc COMMAND test_ball_vmc ${CMAKE_CURRENT_SOURCE_DIR}/test/ball_simulation_normal.tsv)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_sdk_torque_path test/test_sdk_torque_path.cpp)
  target_link_libraries(test_sdk_torque_path eit_vmc_core Franka::Franka)
  add_test(NAME sdk_torque_path COMMAND test_sdk_torque_path)
  add_test(NAME eit_model COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_eit.py)
  add_test(NAME joystick_feedback COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joystick_feedback.py)
  add_test(NAME experiment_block COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_block.py)
  add_test(NAME joystick_recorder COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../joystick_hardware/test_recorder.py)
  add_test(NAME operator_release COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_operator_release.py)
  add_test(NAME joystick_plots COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joystick_plots.py)
  add_test(NAME experiment_recording COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_recording.py)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
