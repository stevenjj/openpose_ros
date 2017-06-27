cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros)

set(OPENPOSE_INCLUDE_DIRS openpose/include 
						  openpose/src 
						  /usr/local/cuda/include 
						  openpose/3rdparty/caffe/distribute/include)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
roslib)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES openpose_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
  ${OPENPOSE_INCLUDE_DIRS}
)

add_definitions(-DUSE_CAFFE)

FIND_LIBRARY(OPENPOSE_LIBRARY openpose openpose/build/lib)
FIND_LIBRARY(CAFFE_LIBRARY caffe openpose/3rdparty/caffe/distribute/lib)
FIND_LIBRARY(CUDA_LIBRARY cuda openpose/build/lib)
FIND_LIBRARY(GFLAGS_LIBRARY gflags /usr/lib/x86_64-linux-gnu)
FIND_LIBRARY(GLOG_LIBRARY glog /usr/lib/x86_64-linux-gnu)

add_executable(${PROJECT_NAME}_talker_node src/talker.cpp)
target_link_libraries(${PROJECT_NAME}_talker_node ${catkin_LIBRARIES} 
												  ${OPENPOSE_LIBRARY} 
												  ${CAFFE_LIBRARY} 
												  ${CUDA_LIBRARY})

add_executable(${PROJECT_NAME}_openpose_node src/openpose.cpp)
target_link_libraries(${PROJECT_NAME}_openpose_node ${catkin_LIBRARIES} 
													${OPENPOSE_LIBRARY} 
													${CAFFE_LIBRARY} 
													${CUDA_LIBRARY} 
													${GFLAGS_LIBRARY} 
													${GLOG_LIBRARY})

