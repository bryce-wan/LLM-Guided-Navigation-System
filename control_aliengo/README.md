## How to build and run
***This code has already prepared on the Aliengo in ~/code/catkin_ws***
### build
```
cd catkin_ws

catkin_make 

```
### run

```
#First activate root permissions, avoiding the memory locked in Aliengo. 

su   

source devel/setup.bash   

#If you have not modified the project name and executable file name of cmake lists   

rosrun traj_to_action walk   


```