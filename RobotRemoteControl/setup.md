# Tracked Robot Environment Setup

## OS Requirement
- You should install Ubuntu 16.04 for udoo first

## Install ROS Kinetic
> note: Ros Kinetic only support Ubuntu 16.04

### 1. Setup your sources.list
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 2. Set up your keys
```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

### 3. Install ROS
First, make sure your system is updated:
```bash
sudo apt-get update
```

Then, install ROS via apt:
```bash
sudo apt-get install ros-kinetic-desktop-full
```

### 4. Initialize rosdep
Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.
```bash
sudo rosdep init
rosdep update
```

### 5. Environment setup
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. Dependencies for building packages
```bash
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Setup tracked_robot project
### 1. Copy the project to udoo
將 Project 複製到 udoo 的家目錄, 移完後檔案結構應如下:
```
~/catkin_ws/
  build/                                <- ros 編譯資料夾, 可刪除
  devel/                                <- ros 編譯資料夾, 可刪除
  src/
    tracked_robot/
      tracked_robot/                    <- robot 的主程式
      dynamixel-workbench/              <- 雲台馬達 ros API
      dynamixel-workbench-msgs/         <- 雲台馬達 ros API
      my_dynamixel_workbench_tutorial/  <- 雲台馬達 ros API
      README.md
    CMakeList.txt
```
### 2. Setup Dynamixel

#### Install main package
two choice to install main package:
1. Install via apt
```bash
sudo apt-get install ros-kinetic-dynamixel-workbench
sudo apt-get install ros-kinetic-dynamixel-workbench-msgs
```
2. Install via git
```bash
cd ~/catkin_ws/src/tracked_robot/tracked_robot/
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git -b kinetic-devel
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git -b kinetic-devel
```

#### Install dependent package
```bash
sudo apt-get install ros-kinetic-dynamixel-sdk
sudo apt-get install ros-kinetic-qt-build
```

#### Setup Position Controller
1. edit `position_control.launch` which is in `~/catkin_ws/src/tracked_robot/my_dynamixel_workbench_tutorial/launch/`. You will see the contents  like:
```xml
<launch>
<arg name="device_name"       default="/dev/head"/>  // edit this line
<arg name="baud_rate"         default="57600"/>

......

</launch>
```

2. Modify `/dev/head` to the correct device name which is your dynamixel, in my case, it is `/dev/ttyUSB1`

3. After edit `position_control.launch`, we need to get the access permission for USB device
```bash
sudo chmod a+rw /dev/ttyUSB1 # replace the device name with yours
```

For more details, please read the reference [dynamixel_workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

### 3. Setup Arduino with ROS ( For Ultrasonic Sensor )
#### Install rosserial_arduino
```bash
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
```

#### check device name
in `~/catkin_ws/src/tracked_robot/tracked_robot/src/ultrasonic.cpp` check the line:
```c++
#define sensorName "/dev/ttymxc3"
```
Modify `sensorName` to which is your device name of the ultrasonic sensor, like `/dev/ttymxc1`, `/dev/ttymxc3`, etc.

For more details, please read the reference

[rosserial_arduino Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

[Ultrasonic-sensor-with-ROS](https://github.com/surabhi96/Library-navigating-robot/wiki/Ultrasonic-sensor-with-ROS)

### 4. Build the project
After install all libraries and dependencies, we can build the project
```
cd ~/catkin_ws
catkin_make
```
