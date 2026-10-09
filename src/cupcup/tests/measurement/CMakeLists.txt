cmake_minimum_required(VERSION 3.10)
project(cupcup_measurement)
set(CMAKE_CXX_STANDARD 14)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(seumath REQUIRED)
find_package(seurobot REQUIRED)
if(NOT RELEASED_CONTROLLER_SOURCE)
  message(FATAL_ERROR "Set RELEASED_CONTROLLER_SOURCE to the extracted released controller/src")
endif()
find_path(WEBOTS_INCLUDE webots/Supervisor.hpp
  HINTS "$ENV{WEBOTS_HOME}/include/controller/cpp" /usr/local/webots/include/controller/cpp REQUIRED)
find_library(WEBOTS_CPP_LIBRARY CppController
  HINTS "$ENV{WEBOTS_HOME}/lib/controller" /usr/local/webots/lib/controller REQUIRED)
add_executable(measurement_supervisor supervisor.cpp
  ${RELEASED_CONTROLLER_SOURCE}/WebotsUtils.cpp)
target_include_directories(measurement_supervisor PRIVATE ${WEBOTS_INCLUDE}
  ${RELEASED_CONTROLLER_SOURCE} ../../src)
target_link_libraries(measurement_supervisor ${WEBOTS_CPP_LIBRARY})
ament_target_dependencies(measurement_supervisor rclcpp common sensor_msgs)
target_compile_options(measurement_supervisor PRIVATE -Wall -Wextra -Wpedantic)
add_executable(test_controller_adapter controller_adapter.cpp
  ${RELEASED_CONTROLLER_SOURCE}/SimRobot.cpp ${RELEASED_CONTROLLER_SOURCE}/WebotsUtils.cpp)
target_include_directories(test_controller_adapter PRIVATE ${WEBOTS_INCLUDE} ${RELEASED_CONTROLLER_SOURCE})
target_link_libraries(test_controller_adapter ${WEBOTS_CPP_LIBRARY})
ament_target_dependencies(test_controller_adapter rclcpp common sensor_msgs seumath)
add_executable(action_templates action_templates.cpp)
ament_target_dependencies(action_templates seurobot common)
file(SHA256 "${RELEASED_CONTROLLER_SOURCE}/supervisor.cpp" RELEASED_SUPERVISOR_SHA)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/traced_supervisor.cpp" TRACE_WRAPPER_SHA)
add_executable(traced_supervisor traced_supervisor.cpp
  ${RELEASED_CONTROLLER_SOURCE}/WebotsUtils.cpp)
target_include_directories(traced_supervisor PRIVATE ${WEBOTS_INCLUDE} ${RELEASED_CONTROLLER_SOURCE})
target_link_libraries(traced_supervisor ${WEBOTS_CPP_LIBRARY})
target_compile_definitions(traced_supervisor PRIVATE
  RELEASED_SUPERVISOR_FILE="${RELEASED_CONTROLLER_SOURCE}/supervisor.cpp"
  RELEASED_SUPERVISOR_SHA="${RELEASED_SUPERVISOR_SHA}"
  TRACE_WRAPPER_SHA="${TRACE_WRAPPER_SHA}")
ament_target_dependencies(traced_supervisor rclcpp common)
