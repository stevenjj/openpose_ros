#!/bin/bash
touch CATKIN_IGNORE
echo "------------------------- Install Script -------------------------"
echo "This will run the original install scripts from openpose."
cd openpose
./install_caffe_and_openpose.sh
cd ../
echo "------------------------- Enabling Package -------------------------"
echo "Assuming everything in openpose built properly, we will now rename ignore_package.xml to package.xml so that catkin can build this package"
rm CATKIN_IGNORE