cmake_minimum_required(VERSION 2.8.3)
project(skeleton_extract_3d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs  
  pcl_ros
  pcl_conversions
  cv_bridge
  message_filters
  openpose_ros_msgs  
  visualization_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  CATKIN_DEPENDS other_catkin_pkg
)

include_directories(
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)


add_executable(skeleton_extract_3d_node src/skeleton_extract_3d_node.cpp)
target_link_libraries(skeleton_extract_3d_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(skeleton_extract_3d_node ${catkin_EXPORTED_TARGETS})

add_executable(skeleton_extract_3d_visualization_node src/skeleton_extract_3d_visualization_node.cpp)
target_link_libraries(skeleton_extract_3d_visualization_node ${catkin_LIBRARIES})
add_dependencies(skeleton_extract_3d_visualization_node ${catkin_EXPORTED_TARGETS})
