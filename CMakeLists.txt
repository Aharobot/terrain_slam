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

# initially
set(EXTERNAL_LIBS "")

#--------------------
# DEPENDENCY: CGAL
#--------------------
# CGAL and its components
find_package( CGAL QUIET COMPONENTS  HINTS "/usr/local/lib/" "C:/Program Files/CGAL-4.3/build/")
message("-- CGAL found, version ${CGAL_VERSION}")
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
# include helper file
include( ${CGAL_USE_FILE} )

#--------------------
# DEPENDENCY: Boost
#--------------------
# Boost and its components
find_package( Boost REQUIRED COMPONENTS filesystem thread program_options system graph)
message("-- Boost found, version ${Boost_VERSION}")
add_definitions("-DBOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT")
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${Boost_LIBRARIES})

#--------------------
# DEPENDENCY: OpenCV
#--------------------
find_package(OpenCV REQUIRED HINTS "/opt/ros/kinetic")
message("-- OpenCV found, version ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OpenCV_LIBRARIES})

#--------------------
# DEPENDENCY: GDAL
#--------------------
find_package(GDAL REQUIRED)
message("-- GDAL found, version ${GDAL_VERSION}")
include_directories(${GDAL_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${GDAL_LIBRARIES})

#--------------------
# DEPENDENCY: GMP
#--------------------
find_package(GMP REQUIRED)
message("-- GMP found, version ${GMP_VERSION}")
include_directories(${GMP_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${GMP_LIBRARIES})

#--------------------
# DEPENDENCY: MPFR
#--------------------
find_package(MPFR REQUIRED)
message("-- MPFR found, version ${MPFR_VERSION}")
include_directories(${MPFR_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${MPFR_LIBRARIES})

#--------------------
# DEPENDENCY: Ceres
#--------------------
find_package(Ceres REQUIRED)
message("-- CERES found, version ${CERES_VERSION}")
include_directories(${CERES_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${CERES_LIBRARIES})

#--------------------
# DEPENDENCY: OpenMP
#--------------------
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message("-- OpenMP found, version ${OpenMP_VERSION}")
endif()

#--------------------
# DEPENDENCY: eigen 3
#--------------------
find_package(Eigen3 REQUIRED)
message("-- eigen3 found, version ${Eigen3_VERSION}")
include_directories(${Eigen3_INCLUDE_DIR})
#note: no library to add, eigen rely only on headers

#--------------------
# DEPENDENCY: nabo
#--------------------
find_package(libnabo REQUIRED)
include_directories(${libnabo_INCLUDE_DIRS})
message("-- libnabo found, version ${libnabo_VERSION}")
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${libnabo_LIBRARIES})

# Workaround for MSCV - show includes in project
FILE(GLOB_RECURSE MY_HEADERS include/*.h)

# include for local directory
include_directories(BEFORE include)

add_library(outlier_remover src/outlier_remover.cpp)
target_link_libraries(outlier_remover ${libnabo_LIBRARIES})

add_library(adjuster src/adjuster.cpp)
target_link_libraries(adjuster ${CERES_LIBRARIES})

add_executable(terrain_slam src/terrain_slam.cpp ${MY_HEADERS})

add_to_cached_list(CGAL_EXECUTABLE_TARGETS terrain_slam)

# Link the executable to CGAL and third-party libraries
target_link_libraries(terrain_slam  ${OpenCV_LIBRARIES}
                                    ${Boost_LIBRARIES}
                                    outlier_remover
                                    adjuster)
