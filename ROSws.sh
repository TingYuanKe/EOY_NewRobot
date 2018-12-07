#!/bin/sh

if [ ! $# -eq 1 ]; then
	echo "Usage:./ROSws.sh <package_name>"
	exit
fi

package=$1
path=$HOME



read -p "Are You Sure to clean catkin_ws ? (y/n) " yn

case $yn in
	[Yy]* ) rm -rf "$path/catkin_ws";catkin_init_workspace ;;
	* ) ;;
esac

mkdir -p "$path/catkin_ws/src"
cd "$path/catkin_ws/src"

echo "**************************************"
echo "***  Create Workspace and package  ***"
echo "**************************************"

case $yn in
	[Yy]* ) catkin_init_workspace ;;
	* ) ;;
esac

catkin_create_pkg $package std_msgs rospy roscpp
cp "$path/catkin_ws/src/$package/CMakeLists.txt" "$path/catkin_ws/src/$package/CMakeLists.txt.orig"
echo 
echo "successfully! Create catkin_ws/src/$package "


