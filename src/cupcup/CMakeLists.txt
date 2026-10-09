cmake_minimum_required(VERSION 3.5)
project(cupcup)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(cupcup src/player.cpp)
target_include_directories(cupcup PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(cupcup ${OpenCV_LIBS})
ament_target_dependencies(cupcup rclcpp common sensor_msgs)

# The standalone strategy sandbox is headless and shares strategy_logic.hpp
# with the ROS/Webots player. The browser UI only visualizes its output.
add_executable(cupcup_sandbox src/tactical_sandbox.cpp)
target_include_directories(cupcup_sandbox PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

install(TARGETS cupcup cupcup_sandbox
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS tools/platform_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cupcup_visualizer)
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools/web
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_compile_options(-UNDEBUG)
  add_executable(navigation_test tests/navigation_test.cpp)
  target_include_directories(navigation_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME navigation_test COMMAND navigation_test)
  add_executable(robot_perception_test tests/robot_perception_test.cpp)
  target_include_directories(robot_perception_test PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(robot_perception_test ${OpenCV_LIBS})
  add_test(NAME robot_perception_test COMMAND robot_perception_test)
  add_executable(tactical_sim_test tests/tactical_sim_test.cpp)
  target_include_directories(tactical_sim_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME tactical_sim_test COMMAND tactical_sim_test)
  add_executable(match_policy_test tests/match_policy_test.cpp)
  target_include_directories(match_policy_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_options(match_policy_test PRIVATE -UNDEBUG)
  add_test(NAME match_policy_test COMMAND match_policy_test)
  add_executable(ball_tracker_test tests/ball_tracker_test.cpp)
  target_include_directories(ball_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME ball_tracker_test COMMAND ball_tracker_test)
  add_executable(strategy_logic_test tests/strategy_logic_test.cpp)
  target_include_directories(strategy_logic_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME strategy_logic_test COMMAND strategy_logic_test)
  add_executable(world_model_test tests/world_model_test.cpp)
  target_include_directories(world_model_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME world_model_test COMMAND world_model_test)
  add_executable(sensor_time_test tests/sensor_time_test.cpp)
  target_include_directories(sensor_time_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME sensor_time_test COMMAND sensor_time_test)
  add_executable(camera_geometry_test tests/camera_geometry_test.cpp)
  target_include_directories(camera_geometry_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME camera_geometry_test COMMAND camera_geometry_test)
  add_executable(ball_appearance_test tests/ball_appearance_test.cpp)
  target_include_directories(ball_appearance_test PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(ball_appearance_test ${OpenCV_LIBS})
  add_test(NAME ball_appearance_test COMMAND ball_appearance_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME measurement_analysis_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/measurement_analysis_test.py)
  add_test(NAME robot_labels_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/robot_labels_test.py)
  add_test(NAME tactical_sandbox_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/tactical_sandbox_test.py
    $<TARGET_FILE:cupcup_sandbox>)
  add_test(NAME platform_server_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/platform_server_test.py
    $<TARGET_FILE:cupcup_sandbox>)
  add_test(NAME world_trace_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/world_trace_test.py)
  add_test(NAME ball_geometry_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/ball_geometry_test.py)
  add_test(NAME match_summary_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/match_summary_test.py)
  add_test(NAME kick_contacts_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/kick_contacts_test.py)
  add_test(NAME image_recording_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/image_recording_test.py)
  add_test(NAME package_contract_test COMMAND
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_package.py)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
