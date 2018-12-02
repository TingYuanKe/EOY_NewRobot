#!/bin/sh

if [ ! $# -eq 2 ]; then
	echo "Usage:./ROSmake.sh <EOY_Repo_path> <package_name>"
	exit
fi

RepoPath=$1

SrcPath="$RepoPath/EyeOnYouDepthCam/doc/NiTE-test/Samples/UserViewer"
IncludePath="$RepoPath/EyeOnYouDepthCam/doc/NiTE-test/Include"
OpenNI_src="$RepoPath/EyeOnYouDepthCam/doc/OpenNI-Linux-x64-2.2/Redist"
NiTE_src="$RepoPath/EyeOnYouDepthCam/doc/NiTE-Linux-x64-2.2/Redist"

package=$2

workspace="$HOME/catkin_ws"
#workspace="$HOME/tt/catkin_ws"
workspace_IncludePath="$workspace/src/$package/include"


# overrided src file
cp "$SrcPath/"* "$workspace/src/$package/src"
echo "Copy Src files to workspace successfully."

# check Include
if [ ! -e "$workspace_IncludePath/OpenNI.h" ]; then
	cp -R "$IncludePath/"* "$workspace_IncludePath"
	echo "Copy Include header files successfully."
fi

# check NiTE2 dir
if [ ! -d "$workspace/src/$package/NiTE2" ]; then
	cp -R "$NiTE_src" "$workspace/src/$package/NiTE2"
	echo "Copy NiTE library successfully."
fi

# check openni dir
if [ ! -d "$workspace/src/$package/OpenNI2" ]; then
	cp -R "$OpenNI_src" "$workspace/src/$package/OpenNI2"
	echo "Copy OpenNI library successfully."
fi

# edit CMakeList
echo "Start to edit CMakeLists.txt."
if [ ! -e "$workspace/src/$package/CMakeLists.txt.orig" ]; then
	echo "CMakeLists.txt.orig does not exist, You need to edit it and compile by yourself."
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

	echo "Edit successfully."
fi

echo "Start to compile"
cd $workspace
catkin_make

 
