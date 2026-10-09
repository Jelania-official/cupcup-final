cmake_minimum_required(VERSION 3.10)
project(cupcup_released_referee_test)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
if(NOT EXISTS "${RELEASED_GAMECTRL_SOURCE}/ctrlwindow.cpp" OR
  NOT EXISTS "${RELEASED_GAMECTRL_SOURCE}/ctrlwindow.hpp")
  message(FATAL_ERROR "Set RELEASED_GAMECTRL_SOURCE to the untouched released gamectrl/src")
endif()
file(SHA256 "${RELEASED_GAMECTRL_SOURCE}/ctrlwindow.cpp" REFEREE_CPP_SHA256)
file(SHA256 "${RELEASED_GAMECTRL_SOURCE}/ctrlwindow.hpp" REFEREE_HPP_SHA256)
add_executable(released_referee operator.cpp
  "${RELEASED_GAMECTRL_SOURCE}/ctrlwindow.cpp"
  "${RELEASED_GAMECTRL_SOURCE}/ctrlwindow.hpp")
target_include_directories(released_referee PRIVATE "${RELEASED_GAMECTRL_SOURCE}")
target_link_libraries(released_referee Qt5::Widgets)
ament_target_dependencies(released_referee rclcpp common)
target_compile_definitions(released_referee PRIVATE
  REFEREE_CPP_SHA256="${REFEREE_CPP_SHA256}"
  REFEREE_HPP_SHA256="${REFEREE_HPP_SHA256}")
target_compile_options(released_referee PRIVATE -Wall -Wextra -Wpedantic)
