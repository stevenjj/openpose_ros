# openpose_ros 
My ROS Wrapper for openpose https://github.com/CMU-Perceptual-Computing-Lab/openpose
(see commit number `a1e0a5f4136e702b5731a268c2993fb75ca4753c`)
# Installing and Testing the Openpose ROS Wrapper
1. Install openpose and its dependencies then enable the package by running
````
./install_openpose_and_enable_package.sh
````
2. If everything succeeds, run `catkin build`, source your workspace then:
````
roscd openpose_ros_pkg/../openpose
rosrun openpose_ros_pkg openpose_ros --image_dir examples/media/
````

3. Go to `src/openpose_ros_node_firephinx.cpp` and edit the lines:
````
// Camera Topic
DEFINE_string(camera_topic,             "/multisense/left/image_rect_color_rotated_180",      "Image topic that OpenPose will process.");
````
Change the string accordingly. camera_topic should be the ros sensor_msgs::Image topic you subscribe to. For example `"/camera/image_raw"`

Run `catkin build` and source your workspace again then run:
````
rosrun openpose_ros_pkg openpose_ros_node_firephinx
````
If this works, you should see an openCV visualization open up:


## Example Service Code
1. To start the ros service run:
````
rosrun openpose_ros_pkg openpose_ros_node 
````
2. To test if the service is working run:
````
rosrun openpose_ros_pkg test_openpose_ros_service_call 
````
and subscribe to `/openpose_ros/input_image` for the input image and `/openpose_ros/detected_poses_image` for the output image




# Similar packages
Naturally, `openpose` is quite popular and similar packages for ros can be found at

https://github.com/tue-robotics/openpose_ros

https://github.com/firephinx/openpose_ros