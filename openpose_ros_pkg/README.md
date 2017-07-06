This package runs openpose as a ros executable. (see src/openpose.cpp)

The exeuctable `openpose_ros_node_firephinx` is based on https://github.com/firephinx/openpose_ros but was slightly changed to make it compilable with this repo's CMakeLists.txt as well as a functionality for finding the model_path automatically using ros package path

The ROS service `openpose_ros_node` was originally based on https://github.com/tue-robotics/openpose_ros but has since been modified to reflect newer changes on openpose.
