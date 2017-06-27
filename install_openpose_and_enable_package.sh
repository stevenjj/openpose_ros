#!/bin/bash
echo "Adding CATKIN_IGNORE to this package. Will remove if install script successfully runs"
touch CATKIN_IGNORE
echo "------------------------- Install Script -------------------------"
echo "This will run the original install scripts from openpose."
cd openpose
./install_caffe_and_openpose.sh
cd ../
echo "------------------------- Enabling Package -------------------------"
echo "Assuming everything in openpose built properly, we remove CATKIN_IGNORE"
rm CATKIN_IGNORE