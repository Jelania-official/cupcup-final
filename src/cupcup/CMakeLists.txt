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

install(TARGETS cupcup
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(ball_tracker_test tests/ball_tracker_test.cpp)
  target_include_directories(ball_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME ball_tracker_test COMMAND ball_tracker_test)
  add_executable(strategy_logic_test tests/strategy_logic_test.cpp)
  target_include_directories(strategy_logic_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME strategy_logic_test COMMAND strategy_logic_test)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
