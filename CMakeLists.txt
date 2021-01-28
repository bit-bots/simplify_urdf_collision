cmake_minimum_required(VERSION 2.8.3)
project(simplify_urdf_collision)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(SOURCES
        src/optimal_bounding_box.cpp
        )


set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(PythonLibs)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    find_package(Boost REQUIRED COMPONENTS python3)
else()
    find_package(Boost REQUIRED COMPONENTS python)
endif()
find_package (PythonLibs COMPONENTS Interpreter Development)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosconsole
        )

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

catkin_python_setup()
enable_bitbots_docs()


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        DEPENDS EIGEN3
)

# create the lib
add_library(py_optimal_bounding_box SHARED src/boundingbox_pywrapper.cpp ${SOURCES})
add_dependencies(py_optimal_bounding_box ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# link
target_link_libraries(py_optimal_bounding_box ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CODE_LIBRARIES})

set_target_properties(py_optimal_bounding_box PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
