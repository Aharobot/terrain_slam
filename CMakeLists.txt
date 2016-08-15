# Created by the script cgal_create_cmake_script_with_options
# This is the CMake script for compiling a set of CGAL applications.
project( terrain_slam C CXX )
cmake_minimum_required(VERSION 2.6.2)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true )
if ( COMMAND cmake_policy )
  cmake_policy( SET CMP0003 NEW )
endif()

# initially
set(EXTERNAL_LIBS "")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options filesystem thread system graph)
add_definitions(-fext-numeric-literals)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(CGAL)
find_package(G2O REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message("-- OpenMP found, version ${OpenMP_VERSION}")
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(Eigen_INCLUDE_DIR ${Eigen3_INCLUDE_DIRS})
set(Boost_USE_MULTITHREADED ON)
include( ${CGAL_USE_FILE} )
include_directories(include
                    ${OpenCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIRS}
                    ${SUITESPARSE_INCLUDE_DIRS}
                    ${Eigen3_INCLUDE_DIR}
                    ${libnabo_INCLUDE_DIRS}
                    ${CHOLMOD_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS})

set(g2o_LIBRARIES
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG}
    ${G2O_SOLVER_SLAM2D_LINEAR}
    ${G2O_SOLVER_STRUCTURE_ONLY}
    ${G2O_SOLVER_EIGEN}
    ${G2O_STUFF_LIBRARY}
    ${G2O_CORE_LIBRARY}
    ${G2O_CLI_LIBRARY}
    ${G2O_TYPES_DATA}
    ${G2O_TYPES_ICP}
    ${G2O_TYPES_SBA}
    ${G2O_TYPES_SCLAM2D}
    ${G2O_TYPES_SIM3}
    ${G2O_TYPES_SLAM2D}
    ${G2O_TYPES_SLAM3D})

# Workaround for MSCV - show includes in project
FILE(GLOB_RECURSE MY_HEADERS include/*.h)

add_library(graph src/graph.cpp)
target_link_libraries(graph ${g2o_LIBRARIES} cholmod)

add_library(greedy_projector src/greedy_projector.cpp)
target_link_libraries(graph ${PCL_LIBRARIES})

add_library(adjuster src/adjuster.cpp)
target_link_libraries(adjuster greedy_projector ${CERES_LIBRARIES})

add_executable(terrain_slam src/terrain_slam.cpp
                            src/clouds.cpp
                            ${MY_HEADERS})

# Link the executable to CGAL and third-party libraries
target_link_libraries(terrain_slam  ${Boost_LIBRARIES}
                                    ${OpenCV_LIBRARIES}
                                    ${SUITESPARSE_LIBRARIES}
                                    ${g2o_LIBRARIES}
                                    ${PCL_LIBRARIES}
                                    cholmod
                                    adjuster
                                    graph)
set_property(TARGET adjuster PROPERTY CXX_STANDARD 11)
set_property(TARGET adjuster PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET graph PROPERTY CXX_STANDARD 11)
set_property(TARGET graph PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET terrain_slam PROPERTY CXX_STANDARD 11)
set_property(TARGET terrain_slam PROPERTY CXX_STANDARD_REQUIRED ON)

add_executable(evaluate_adjuster src/evaluate_adjuster.cpp
                                 src/clouds.cpp
                                 ${MY_HEADERS})
target_link_libraries(evaluate_adjuster adjuster
                                        graph
                                        ${PCL_LIBRARIES}
                                        ${Boost_LIBRARIES})
set_property(TARGET evaluate_adjuster PROPERTY CXX_STANDARD 11)
set_property(TARGET evaluate_adjuster PROPERTY CXX_STANDARD_REQUIRED ON)

add_executable(test_adjuster src/test_adjuster.cpp
                             src/greedy_projector.cpp
                             src/clouds.cpp
                             ${MY_HEADERS})
target_link_libraries(test_adjuster adjuster
                                    graph
                                    ${PCL_LIBRARIES}
                                    ${Boost_LIBRARIES})
set_property(TARGET test_adjuster PROPERTY CXX_STANDARD 11)
set_property(TARGET test_adjuster PROPERTY CXX_STANDARD_REQUIRED ON)
