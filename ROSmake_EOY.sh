#!/bin/sh

if [ ! $# -eq 2 ]; then
	echo "Usage:./ROSmake.sh <EOY_Repo_path> <package_name>"
	exit
fi

RepoPath=$1

SrcPath="$RepoPath/EyeOnYouDepthCam/exec/src"
IncludePath="$RepoPath/EyeOnYouDepthCam/Include"
OpenNI_src="$RepoPath/EyeOnYouDepthCam/lib/OpenNI-Linux-x64-2.2/Redist"
NiTE_src="$RepoPath/EyeOnYouDepthCam/lib/NiTE-Linux-x64-2.2/Redist"

package=$2

workspace="$HOME/catkin_ws"
workspace_IncludePath="$workspace/src/$package/include"


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
cp "$SrcPath/"* "$workspace/src/$package/src"
echo "${GREEN}Copy Src files to workspace successfully."${NC}

# check Include
if [ ! -e "$workspace_IncludePath/OpenNI.h" ]; then
	cp -R "$IncludePath/"* "$workspace_IncludePath"
	echo "${GREEN}Copy Include header files successfully.${NC}"
fi

# check NiTE2 dir
if [ ! -d "$workspace/src/$package/NiTE2" ]; then
	cp -R "$NiTE_src" "$workspace/src/$package/NiTE2"
	echo "${GREEN}Copy NiTE library successfully.${NC}"
fi

# check openni dir
if [ ! -d "$workspace/src/$package/OpenNI2" ]; then
	cp -R "$OpenNI_src" "$workspace/src/$package/OpenNI2"
	echo "${GREEN}Copy OpenNI library successfully.${NC}"
fi

# edit CMakeList
if [ ! -e "$workspace/src/$package/CMakeLists.txt.orig" ]; then
	echo "${RED}CMakeLists.txt.orig does not exist, You need to edit it and compile by yourself.${NC}"
	exit
else
	# copy new CmakeList.txt
	cp "$workspace/src/$package/CMakeLists.txt.orig" "$workspace/src/$package/CMakeLists.txt"
	
	# add compile option
	sed -i 's/c++11)/c++11)\nadd_compile_options(-lpthread)\n/' "$workspace/src/$package/CMakeLists.txt" 

	# add find_package
	sed -i 's/system)/system)\n\nfind_package(OpenGL REQUIRED)\nfind_package(GLUT REQUIRED)\n/' "$workspace/src/$package/CMakeLists.txt"
	
	# add include_directories
	sed -i 's/# include/  include/' "$workspace/src/$package/CMakeLists.txt"
	sed -i 's/${catkin_INCLUDE_DIRS}/${catkin_INCLUDE_DIRS}\n  ${OPENGL_INCLUDE_DIRS}\n  ${GLUT_INCLUDE_DIRS}/' "$workspace/src/$package/CMakeLists.txt"
	
	# add link_directories
	sed -i 's/## Declare a C++ executable/# Setting link directories\nlink_directories(\n  NiTE2\n  OpenNI2\n)\n\n\n## Declare a C++ executable/' "$workspace/src/$package/CMakeLists.txt"
	
	# add executable file
	cpp_file=$(ls "$workspace/src/$package/src/"*.cpp)
	add_config="add_executable(\${PROJECT_NAME}_node src/main.cpp $(echo $cpp_file | sed  's,'"$workspace\/src\/$package"\/,',g' | sed 's/src\/stdafx.cpp //' | sed 's/src\/main.cpp //' ))"
	sed -i 's,node.cpp),node.cpp)\n '"$add_config"'\n,' "$workspace/src/$package/CMakeLists.txt"
	
	# add target_link_libraries
	sed -i 's/against/against\ntarget_link_libraries(${PROJECT_NAME}_node\n  ${catkin_LIBRARIES}\n  ${OPENGL_LIBRARIES}\n  ${GLUT_LIBRARY}\n  OpenNI2\n  NiTE2\n)\n/' "$workspace/src/$package/CMakeLists.txt"

	echo "${GREEN}Edit CMakeLists.txt successfully.${NC}"
fi

echo
echo "**************************************"
echo "***       Start to compile         ***"
echo "**************************************"


cd $workspace
catkin_make

 # copy NiTE2 dir to catkin_ws for quickly use
if [ ! -d "$workspace/devel/NiTE2" ];then
	cp -R "$workspace/src/$package/NiTE2/NiTE2" "$workspace/devel"
	echo "${GREEN}Copy NiTE2 directory to catkin_ws successfully.${NC}"
fi

