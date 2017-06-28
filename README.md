# Using OPENPOSE Wrapper
1. Install openpose and its dependencies then enable the package by running
````
./install_openpose_and_enable_package.sh
````
2. If everything succeeds, run `catkin build`, source your workspace then:
````
roscd openpose_ros/openpose
rosrun openpose_ros openpose_ros --image_dir examples/media/
````

