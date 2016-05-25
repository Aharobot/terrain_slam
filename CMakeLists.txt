# Created by the script cgal_create_cmake_script_with_options
# This is the CMake script for compiling a set of CGAL applications.

project( terrain_slam )

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

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  HINTS "/usr/local/lib/" "C:/Program Files/CGAL-4.3/build/")
if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()
endif()

# include helper file
include( ${CGAL_USE_FILE} )

# Boost and its components
find_package( Boost REQUIRED COMPONENTS filesystem thread program_options system graph)
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()
add_definitions("-DBOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT")

find_package(OpenCV REQUIRED)
if ( NOT OpenCV_FOUND )
  message(STATUS "This project requires the OpenCV library, and will not be compiled.")
  return()
endif()

find_package(GDAL REQUIRED)
if ( NOT GDAL_FOUND )
  message(STATUS "This project requires the GDAL library, and will not be compiled.")
  return()
endif()

find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

FILE(GLOB_RECURSE MY_HEADERS include/*.h)

# include for local directory
include_directories( BEFORE include ${GMP_INCLUDE_DIRS}
                                    ${OpenCV_INCLUDE_DIRS}
                                    ${MPFR_INCLUDE_DIRS})

add_executable(terrain_slam src/terrain_slam.cpp
                            ${MY_HEADERS})

add_to_cached_list( CGAL_EXECUTABLE_TARGETS terrain_slam )

# # Link the executable to CGAL and third-party libraries
target_link_libraries(terrain_slam ${CGAL_LIBRARIES}
                                        ${CGAL_3RD_PARTY_LIBRARIES}
                                        ${GDAL_LIBRARIES}
                                        ${GMP_LIBRARIES}
                                        ${MPFR_LIBRARIES}
                                        ${OpenCV_LIBRARIES}
                                        ${Boost_LIBRARIES})

