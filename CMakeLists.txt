# Created by the script cgal_create_cmake_script_with_options
# This is the CMake script for compiling a set of CGAL applications.
project( terrain_slam CXX )

cmake_minimum_required(VERSION 2.6.2)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

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

# find_package(CGAL QUIET COMPONENTS  HINTS "/usr/local/lib/" "C:/Program Files/CGAL-4.3/build/")
find_package(Boost REQUIRED COMPONENTS filesystem thread program_options system graph)
find_package(OpenCV REQUIRED HINTS "/opt/ros/kinetic")
# find_package(GDAL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP)
find_package(G2O REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)
find_package(libnabo REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message("-- OpenMP found, version ${OpenMP_VERSION}")
endif()

# include helper file
# include( ${CGAL_USE_FILE} )

# ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES}
# ${GDAL_LIBRARIES}


include_directories(${OpenCV_INCLUDE_DIRS}
                    # ${GDAL_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIRS}
                    ${SUITESPARSE_INCLUDE_DIRS}
                    ${Eigen3_INCLUDE_DIR}
                    ${libnabo_INCLUDE_DIRS}
                    ${CHOLMOD_INCLUDE_DIR})

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

# include for local directory
include_directories(BEFORE include)

add_library(outlier_remover src/outlier_remover.cpp)
target_link_libraries(outlier_remover ${libnabo_LIBRARIES})

add_library(adjuster src/adjuster.cpp)
target_link_libraries(adjuster ${CERES_LIBRARIES})

add_library(gridder src/gridder.cpp)
target_link_libraries(gridder ${CERES_LIBRARIES})

add_library(graph src/graph.cpp)
target_link_libraries(graph ${g2o_LIBRARIES})

add_executable(terrain_slam src/terrain_slam.cpp
                            src/clouds.cpp
                            ${MY_HEADERS})

# add_to_cached_list(CGAL_EXECUTABLE_TARGETS terrain_slam)

# Link the executable to CGAL and third-party libraries
target_link_libraries(terrain_slam  ${OpenCV_LIBRARIES}
                                    ${Boost_LIBRARIES}
                                    ${SUITESPARSE_LIBRARIES}
                                    ${g2o_LIBRARIES}
                                    cholmod
                                    outlier_remover
                                    adjuster
                                    graph
                                    gridder)
set_property(TARGET graph PROPERTY CXX_STANDARD 11)
set_property(TARGET graph PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET terrain_slam PROPERTY CXX_STANDARD 11)
set_property(TARGET terrain_slam PROPERTY CXX_STANDARD_REQUIRED ON)