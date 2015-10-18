# Copyright (c) 2015, Julian Straub <jstraub@csail.mit.edu>
# Licensed under the MIT license. See the license file LICENSE.

project(dpvMFoptRot)
cmake_minimum_required(VERSION 2.8.1)
set(POD_NAME dpvMFoptRot)
include(cmake/pods.cmake)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_COLOR_MAKEFILE ON)
IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release) # Release; debug; ...
  set(CMAKE_BUILD_TYPE Debug) # Release; debug; ...
ENDIF()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )
endif()

find_package (Eigen3 REQUIRED)
find_package (CUDA REQUIRED)
find_package (Boost 1.54 REQUIRED)
find_package (OpenCV REQUIRED)
find_package (PCL REQUIRED)

# add flags depending on build type
set(CMAKE_CXX_FLAGS_RELEASE "-g -Wall -Wno-deprecated -std=c++11 -O2 -finline-functions -DNDEBUG")
#set(CMAKE_CXX_FLAGS_RELEASE "-g -Wall -Wno-deprecated -O2 -finline-functions")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wno-deprecated -std=c++11")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_20,code=sm_20 -gencode arch=compute_30,code=sm_30 -gencode arch=compute_35,code=\"sm_35,compute_35\")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${CUDA_TOOLKIT_INCLUDE}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

cuda_add_executable(dpvMFoptRot ./src/dp_vmf_opt_rot.cpp)
target_link_libraries(dpvMFoptRot optRot
    jsCore dpMMlowVar cudaPcl
    boost_system boost_thread boost_program_options
    pcl_common pcl_io pcl_visualization pcl_filters pcl_search
    pcl_features
#    vtkCommon vtkFiltering vtkRendering vtkGraphics
    ${OpenCV_LIBS}
  )

pods_install_executables(dpvMFoptRot)
#pods_install_libraries(optRot)
pods_install_headers(
  ./include/dpvMFoptRot/dp_vmf_opt_rot.h
  DESTINATION dpvMFoptRot)
