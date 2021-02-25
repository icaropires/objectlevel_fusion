cmake_minimum_required(VERSION 3.5)
project(fusion_layer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(object_model_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


# ============== Setting as lib ==============

# Setting as lib to reference in testing

file(GLOB SOURCES "src/*.cpp")

add_library(fusion_lib ${SOURCES})

ament_target_dependencies(fusion_lib rclcpp object_model_msgs Eigen3)

target_include_directories(fusion_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# =============================================


# ========= Using lib to build bin =========

add_executable(fusion src/fusion.cpp)
target_link_libraries(fusion fusion_lib)

install(TARGETS
  fusion
  DESTINATION lib/${PROJECT_NAME}) 
# ==========================================


# ============== Checkings ==============

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_alignment test/test_alignment.cpp)
  target_link_libraries(test_alignment fusion_lib)
endif()
# ======================================


ament_package()
