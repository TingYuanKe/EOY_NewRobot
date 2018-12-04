# EOY_NewRobot 

This project is build for AI demo in NCTU.

## **Usage:**

### In UDOO (lower) (ubuntu 16.04):
```
# get setup file ( can write in the .bashrc )
source ~/catkin_ws/devel/setup.bash

# run ros master and all contoller_node
roslauch tracked_robot all_in_one.launch

# only run certain node
roscore

// 雲台馬達 
roslaunch my_dynamixel_workbench_tutorial position_control.launch
// 履帶馬達
rosrun tracked_robot Motor_node
// 超音波
rosrun tracked_robot ultrasonic
```
Note:

You can `export ROS_MASTER_URI=http://udoo:11311` in ~/.bashrc, or define it when everytimes running.


### In Cubi:
```
# run ROSws.sh to create catkin_ws and package < optional >
./ROSws.sh <package_name>

# run ROSmake.sh to copy src and include files from repo, and auto compile < optional >
# more detail for ROSws.sh and ROSmake.sh is in EyeOnYouDepthCam folder's README.md
./ROSmake.sh <EOY_Repo_path> <package_name>

# start to run

# get setup file ( can write in the .bashrc )
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws
rosrun <package_name> <node_name>
```
