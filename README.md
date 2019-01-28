# EOY_NewRobot 

This project is build for AI demo in NCTU. and this project is based on the ROS (Robot Operating System).
After doing PID, the c++ server would parse the PID result and send the Tracking instruction to UDOO with ROS.

It contain 4 different modules: 

* **Device Position** : Android App to sensing the right wrist joint data and send to server via socket.
* **EoyOnYouServer** : Java application that Perform Data Sensning, **Sysnchronization** along with **PID Algorithm** via Socket
* **EoyOnYouDepthCam** : ROS wrapped C++ Robot Vision module that perform Skeleton sensing, histgram sesning, socket clinet IPC, Robot Tracking.

* **tracked_robot** : TODO

## **System Archtecture**
![Imgur](https://i.imgur.com/R2Nfq4r.png)


## **Usage:**

### In UDOO (lower) (ubuntu 16.04):

*Get setup file ( can write in the .bashrc )*

`source ~/catkin_ws/devel/setup.bash`

*(First choice)*

*Run ros master and all contoller_node* 

`roslauch tracked_robot all_in_one.launch`

*(Second choice)*

*Only run certain node*

`roscore`

*雲台馬達(optional)*

`roslaunch my_dynamixel_workbench_tutorial position_control.launch`

*履帶馬達(optional)*

`rosrun tracked_robot Motor_node`

*超音波(optional)*

`rosrun tracked_robot ultrasonic`


Note: You can `export ROS_MASTER_URI=http://udoo:11311` in ~/.bashrc, or define it when everytimes running.


### In Cubi:

*Download repo*

`git clone https://github.com/TingYuanKe/EOY_NewRobot.git`

`cd EOY_NewRobot`

*Run ROSws.sh to create catkin_ws and package, and package_name is the name of the ROS package.*

`./ROSws.sh <package_name>`

*Run ROSmake_EOY.sh to copy EOY src and include files from repo, and can choose to auto compile. 
EOY_Repo_path is the path of the EOY repo which is download from github.
The name of package need to be same as you define in ROSws.sh*

`./ROSmake_EOY.sh <EOY_Repo_path> <package_name>`

*( optional ) Run ROSmake_Manual.sh to copy ROS manual_control src from repo, and can choose to auto compile.
EOY_Repo_path is the path of the EOY repo which is download from github.
The name of package need to be same as you define in ROSws.sh

`./ROSmake_Manual.sh <EOY_Repo_path> <package_name>`

Note: More detail for ROS script is in ROS_srcipt_doc.md 

## Excution

#### Setup Robot and environment

 Make sure you've already set up the Robot and all the devices are connected to the same AP.
 
 You can see the following link to setup all the modules
 * [EyeOnYouDepthCam](https://github.com/TingYuanKe/EOY_NewRobot/tree/master/EyeOnYouDepthCam)
 * [DevicePosition](https://github.com/TingYuanKe/EOY_NewRobot/tree/master/DevicePosition)
 * [EyeOnYouServer](https://github.com/TingYuanKe/EOY_NewRobot/tree/master/EyeOnYouServer)
 * [tracked_robo]


#### Launch Ros node on CUBI

*get setup file ( can write in the .bashrc )*

`source ~/catkin_ws/devel/setup.bash`

`rosrun <package_name> <node_name>`

Note: You can `export ROS_MASTER_URI=http://udoo:11311` in ~/.bashrc, or define it when everytimes running.

####  Launch the 'APP' on the android devices

####  Launch the Server  

