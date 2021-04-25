cmake_minimum_required(VERSION 3.5)
project(fusion_layer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_BUILD_TYPE Release)

find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(object_model_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(CGAL QUIET COMPONENTS)

if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

# ============== Setting as lib ==============

# Setting as lib to reference in testing

file(GLOB SOURCES "src/*.cpp")

add_library(fusion_lib ${SOURCES})

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/RegisterSensor.srv"
    "srv/RemoveSensor.srv"
 )

ament_target_dependencies(fusion_lib rclcpp object_model_msgs Eigen3 CGAL)
rosidl_target_interfaces(fusion_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
