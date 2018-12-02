#!/bin/sh

if [ ! $# -eq 1 ]; then
	echo "Usage:./ROSws.sh <package_name>"
	exit
fi

read -p "Are You Sure to clean catkin_ws ? " yn

case $yn in
	[Yy]* ) ;;
	* ) exit;;
esac

package=$1

path=$HOME
#path="$HOME/tt"

rm -rf "$path/catkin_ws"

echo "**************************************"
echo "***  Create Workspace and package  ***"
echo "**************************************"

mkdir -p "$path/catkin_ws/src"
cd "$path/catkin_ws/src"
catkin_init_workspace
catkin_create_pkg $package std_msgs rospy roscpp
cp "$path/catkin_ws/src/$package/CMakeLists.txt" "$path/catkin_ws/src/$package/CMakeLists.txt.orig"


