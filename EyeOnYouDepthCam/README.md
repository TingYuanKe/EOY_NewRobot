# EOYDepthCam
This module is for Robot depth camera, skeleton sensing, histogram sensin,  socket clinet IPC, PID command and robot tracking.

**IMU data folder **: ~/Data/IMUdata

## Installation

### 1. Install ROS-kinectic

**Setup keys**

installation

Setup bashrc for ROS

Create ROS workspace

Setup bashrc for ROS workspace


### 2. OpenNI2
First, Install** libsusb-1.0 ** and all the package you need.

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

複製library至/usr/lib

```
$ cd OpenNI-Linux-x64-2.2/Redist
$ sudo cp libOpenNI2.jni.so libOpenNI2.so /usr/lib
$ sudo cp -r OpenNI2 /usr/lib
$ ldconfig
```


### 3. Install NiTE2

 **usbutils**
```
sudo apt-get install usbutils
```

[NiTE2](https://osdn.net/projects/sfnet_roboticslab/downloads/External/nite/NiTE-Linux-x64-2.2.tar.bz2/) download here  [參考資料](https://codeyarns.com/2015/08/04/how-to-install-and-use-openni2/) 

```
$ tar -xjvf NiTE-Linux-x64-2.2.tar.bz2
$ cd NiTE-Linux-x64-2.2
$ sudo ./install.sh
```


use ```$ lsusb``` to make sure the d-camera is on duty, than run

```
$ cd PATH/NiTE-Linux-x64-2.2/Samples/Bin 
$ ./UserViewer
```

複製library至/usr/lib
```
$ cd NiTE-Linux-x64-2.2-test/Redist
$ sudo cp libNiTE2.so libNiTE2.jni.so NiTE.ini  /usr/lib
$ sudo cp -r NiTE2 /usr/lib 
$ ldconfig
```

### 4. skeleton tracker via ROS
[參考資料](https://blog.csdn.net/youngpan1101/article/details/71118170) 


### 5. NiTE project build flow
[參考資料](https://github.com/keetsky/NiTE-2.0.0)

