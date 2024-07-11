#! /bin/bash
gnome-terminal --window -- bash -c "rosrun ORB_SLAM2  RGBD  Vocabulary/ORBvoc.txt Examples/RGB-D/realsense_435_go2.yaml"
sleep 1s
gnome-terminal --window -- bash -c "roslaunch octomaptransform.launch"
sleep 1s
gnome-terminal --window -- bash -c "rviz"


