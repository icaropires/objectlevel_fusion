cmake_minimum_required(VERSION 3.5)
project(fusion_layer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(object_model_msgs REQUIRED)

add_executable(fusion src/fusion.cpp)
ament_target_dependencies(fusion rclcpp object_model_msgs)

install(TARGETS
  fusion
  DESTINATION lib/${PROJECT_NAME}) 

target_include_directories(fusion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


if(BUILD_TESTING)
  # TODO: setup tests
  # find_package(ament_gtest)
  # ament_add_gtest(<tests>)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
