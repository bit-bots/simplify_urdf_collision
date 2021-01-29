cmake_minimum_required(VERSION 2.8.3)
project(simplify_urdf_collision)

# todo: correct ros deps
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

set(SOURCES src/optimal_bounding_box.cpp)

set(CGAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CGAL-5.2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set(Boost_USE_STATIC_LIBS OFF)
# set(Boost_USE_MULTITHREADED ON)
# set(Boost_USE_STATIC_RUNTIME OFF)

# load PythonLibs
find_package(PythonLibs COMPONENTS Interpreter Development)

# load correct boost python
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    find_package(Boost REQUIRED COMPONENTS python3)
else()
    find_package(Boost REQUIRED COMPONENTS python)
endif()

find_package(CGAL REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

include(${CGAL_USE_FILE})
include(CGAL_Eigen_support)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)


#catkin_package(
#        LIBRARIES ${PROJECT_NAME}
#        CATKIN_DEPENDS
#        DEPENDS EIGEN3
#
#)

# create the lib
add_library(py_optimal_bounding_box SHARED ${SOURCES})
add_dependencies(py_optimal_bounding_box ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# link
target_link_libraries(py_optimal_bounding_box ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES} CGAL::Eigen_support)

set_target_properties(py_optimal_bounding_box PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

add_executable(optimal_bounding_box "src/optimal_bounding_box.cpp")
target_link_libraries(optimal_bounding_box CGAL::Eigen_support)

catkin_python_setup()
