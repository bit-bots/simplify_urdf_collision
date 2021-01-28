# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.

cmake_minimum_required(VERSION 3.1...3.15)
project(simplify_urdf_collision)

set(CGAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CGAL-5.2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(CGAL REQUIRED)

include(${CGAL_USE_FILE})

find_package(Eigen3 3.1.0 REQUIRED) #(3.1.0 or greater)
include(CGAL_Eigen_support)
if(NOT TARGET CGAL::Eigen_support)
  message(
    STATUS "This project requires the Eigen library, and will not be compiled.")
  return()
endif()

add_executable(obb "src/optimal_bounding_box.cpp")
target_link_libraries(obb CGAL::Eigen_support)
