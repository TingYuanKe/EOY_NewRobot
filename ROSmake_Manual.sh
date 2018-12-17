#!/bin/sh

if [ ! $# -eq 2 ]; then
	echo "Usage:./ROSmake.sh <EOY_Repo_path> <package_name>"
	exit
fi

RepoPath=$1

SrcPath="$RepoPath/RobotRemoteControl"

package=$2

workspace="$HOME/catkin_ws"


RED='\033[0;31m'
NC='\033[0m' # No Color
GREEN='\033[0;32m'

echo "**************************************"
echo "***  Copy Src/Include to workspace ***"
echo "**************************************"
 # check package existed
if [ ! -d "$workspace/src/$package" ];then
	echo "${RED}Package not existed, please check your input package_name.${NC}"
	exit
fi


# overrided src file
cp "$SrcPath/"*.cpp "$workspace/src/$package/src"
cp "$SrcPath/"*.h "$workspace/src/$package/src"
echo "${GREEN}Copy Src files to workspace successfully."${NC}


# edit CMakeList
if [ ! -e "$workspace/src/$package/CMakeLists.txt.orig" ]; then
	echo "${RED}CMakeLists.txt.orig does not exist, You need to edit it and compile by yourself.${NC}"
	exit
else
	# copy new CmakeList.txt
	cp "$workspace/src/$package/CMakeLists.txt.orig" "$workspace/src/$package/CMakeLists.txt"
	
	# add executable file
	cpp_file=$(ls "$workspace/src/$package/src/"*.cpp)
	add_config="add_executable(\${PROJECT_NAME}_node src/main.cpp)"
	sed -i 's,node.cpp),node.cpp)\n '"$add_config"'\n,' "$workspace/src/$package/CMakeLists.txt"
	
	# add target_link_libraries
	sed -i 's/against/against\ntarget_link_libraries(${PROJECT_NAME}_node\n  ${catkin_LIBRARIES}\n)\n/' "$workspace/src/$package/CMakeLists.txt"

	echo "${GREEN}Edit CMakeLists.txt successfully.${NC}"
fi

echo
echo "**************************************"
echo "***       Start to compile         ***"
echo "**************************************"


cd $workspace
catkin_make

