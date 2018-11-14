# EOYDepthCam
This module is for Robot depth camera, skeleton sensing, histogram sensin,  socket clinet IPC, PID command and robot tracking.

**IMU data folder **: ~/Data/IMUdata

## Installation

### 1. Install ROS

**Setup keys**

installation

Setup bashrc for ROS

Create ROS workspace

Setup bashrc for ROS workspace


### 2. OpenNI2
Install** libsusb-1.0 **First

```
$ sudo apt-get install g++ python libusb-1.0-0-dev libudev-dev openjdk-6-jdk freeglut3-dev doxygen graphviz
```
若在使用apt-get下載包的時候出現伺服器的error到 ```/etc/apt``` 輸入
```
sudo sed -i 's/tw.archive.ubuntu.com/ubuntu.cs.nctu.edu.tw/g' /etc/apt/sources.list
```
就會更換apt-get下載伺服器的位置

**Install OpenNi2**

```
$ git clone https://github.com/occipital/OpenNI2.git
$ cd [dir-to-OpenNI2]
$ make
$ cd [dir-to-OpenNI2]/Bin/x64-Release
$ sudo ./NiViewer   （没有sudo会导致找不到USB driver）
```
open ./Niviewer to make sure you have successfully installed

### 3. Install NiTE2
[NiTE2](https://osdn.net/projects/sfnet_roboticslab/downloads/External/nite/NiTE-Linux-x64-2.2.tar.bz2/) download here

```
$ tar -xjvf NiTE-Linux-x64-2.2.tar.bz2
$ cd NiTE-Linux-x64-2.2
$ sudo ./install.sh
```

download  **usbutils**
```
sudo apt-get install usbutils
```

use ```$ lsusb``` to make sure the d-camera is on duty


