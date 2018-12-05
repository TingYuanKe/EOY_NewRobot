# EOY_NewRobot 

This project is build for AI demo in NCTU.

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

*雲台馬達*
`roslaunch my_dynamixel_workbench_tutorial position_control.launch`
*履帶馬達*
`rosrun tracked_robot Motor_node`
*超音波*
`rosrun tracked_robot ultrasonic`


Note: You can `export ROS_MASTER_URI=http://udoo:11311` in ~/.bashrc, or define it when everytimes running.


### In Cubi:

*Download repo*
`git clone https://github.com/TingYuanKe/EOY_NewRobot.git`

`cd EOY_NewRobot`

*Run ROSws.sh to create catkin_ws and package, and package_name is the name of the ROS package.*
`./ROSws.sh <package_name>`

*Run ROSmake.sh to copy src and include files from repo, and auto compile. 
EOY_Repo_path is the path of the EOY repo which is download from github.
The name of package need to be same as you define in ROSws.sh*
`./ROSmake.sh <EOY_Repo_path> <package_name>`

Note: More detail for ROSws.sh and ROSmake.sh is in ROS_srcipt_doc.md 


*Start to run*

*get setup file ( can write in the .bashrc )*
`source ~/catkin_ws/devel/setup.bash`

`rosrun <package_name> <node_name>`

Note: You can `export ROS_MASTER_URI=http://udoo:11311` in ~/.bashrc, or define it when everytimes running.

