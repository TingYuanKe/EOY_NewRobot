# EOYDepthCam
This module is for Robot depth camera, skeleton sensing, histogram sensin,  socket clinet IPC, PID command and robot tracking.

### Package
* OpenNi2
* NiTE2
* OpenCV2
* ROS
* g++ version 7.0

**Wearable sensor data folder**: ~/data/IMUdata

**Depth camera data folder**: ~/data/SkeletonData

## Install Instruction

#### 1. Install ROS-kinectic
**Setup sources.lst**

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

**Setup keys**
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
**installation**
```
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop

$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep update

$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

**Setup bashrc for ROS**
```
$ Add to .bashrc: source /opt/ros/kinetic/setup.bash
```
**Create ROS workspace**
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws/
$ catkin_make
```

**Setup bashrc for ROS workspace**
```
$ Add to .bashrc: source ~/catkin_ws/devel/setup.bash
```

#### 2. OpenNI2
First, Install** libsusb-1.0 ** and all the package you need.

```
sudo apt-get install -y libopenni2-dev openni2-utils
```

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
$ sudo ./NiViewer
```
open ./Niviewer to make sure you have successfully installed

複製library至/usr/lib

```
$ cd OpenNI-Linux-x64-2.2/Redist
$ sudo cp libOpenNI2.jni.so libOpenNI2.so /usr/lib
$ sudo cp -r OpenNI2 /usr/lib
$ ldconfig
```

#### 3. Install NiTE2

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

#### 4. Install OpenCV2
```
#### Dependencies
sudo apt-get update
sudo apt-get install -y build-essential cmake libgtk2.0-dev pkg-config \
                        python-numpy python-dev libavcodec-dev libavformat-dev \
                        libswscale-dev libjpeg-dev libpng12-dev libtiff5-dev \
                        libjasper-dev libopencv-dev checkinstall pkg-config \
                        yasm libjpeg-dev libjasper-dev libavcodec-dev \
                        libavformat-dev libswscale-dev libdc1394-22-dev \
                        libxine2 libgstreamer0.10-dev  libv4l-dev \
                        libgstreamer-plugins-base0.10-dev python-dev \
                        python-numpy libtbb-dev libqt4-dev libgtk2.0-dev \
                        libmp3lame-dev libopencore-amrnb-dev \
                        libopencore-amrwb-dev libtheora-dev libvorbis-dev \
                        libxvidcore-dev x264 v4l-utils

### Download opencv-2.4.13.5
wget https://github.com/opencv/opencv/archive/2.4.13.5.zip -O opencv-2.4.13.5.zip
unzip opencv-2.4.13.5.zip
cd opencv-2.4.13.5
mkdir release
cd release

### Compile and install
cmake -G "Unix Makefiles" -DCMAKE_CXX_COMPILER=/usr/bin/g++ CMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local -DWITH_TBB=ON -DBUILD_NEW_PYTHON_SUPPORT=ON -DWITH_V4L=ON -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON -DBUILD_EXAMPLES=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DBUILD_FAT_JAVA_LIB=ON -DINSTALL_TO_MANGLED_PATHS=ON -DINSTALL_CREATE_DISTRIB=ON -DINSTALL_TESTS=ON -DENABLE_FAST_MATH=ON -DWITH_IMAGEIO=ON -DBUILD_SHARED_LIBS=OFF -DWITH_GSTREAMER=ON ..
make all -j$(nproc) # Uses all machine cores
sudo make install

cd ../../
rm -rf ./opencv-2.4.13.5

sudo apt-get install python-opencv -y

### Echoes OpenCV installed version if installation process was successful
echo -e "OpenCV version:"
pkg-config --modversion opencv
```

## gcc version update (optional)
The default g++ version in ubuntu 16.04 is gcc version 5.4, but in this repo it needs g++ version greater than 7 for some case.
```
# install  add-apt-repository command
sudo apt-get install software-properties-common

# add repo of gcc/g++
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update

# install g++-7
sudo apt-get install g++-7
sudo ln -s /usr/bin/g++-7 /usr/bin/g++ -f
sudo ln -s /usr/bin/gcc-7 /usr/bin/gcc -f


```


## Execution

// TODO




