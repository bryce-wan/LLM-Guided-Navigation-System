echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/home/penghaocheng/env/miniconda3/envs/slam/bin/python
make -j
