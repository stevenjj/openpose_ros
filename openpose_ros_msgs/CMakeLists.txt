cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	std_msgs
	sensor_msgs
	message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   BodypartDetection.msg
   PersonDetection.msg
   BodypartDetection_3d.msg
   PersonDetection_3d.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPersons.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(CATKIN_DEPENDS 
					message_runtime 
					std_msgs
					sensor_msgs)
