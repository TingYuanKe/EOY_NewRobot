## Auto create ROS workspace and package
#### ROSws.sh<span></span>
**Usage :**
```
./ROSws.sh <package_name>
```
package_name : the name of the ROS package.

Default path of workspace is at your $HOME, you can change it at variable $path.

**Description :** 

It would auto-create the catkin_ws, before creating a new workspace, it would remove the existed one.
After creating package, it would copy the original CMakeLists.txt as CMakeLists.txt.orig to use for auto-create CMakeLists.txt in ROSmake.sh<span></span>.

## Auto copy src file and auto compile
#### ROSmake.sh<span></span>
**Usage :**
```
./ROSmake.sh <EOY_Repo_path> <package_name>"
```

EOY_Repo_path: The path of the EOY repo which is download from github.
package_name : The name of the ROS package.

Default path of workspace is at your ```$HOME/catkin_ws```, you can change it at variable ```$workspace``` in Line 17.

Default source code path in repo is ```$RepoPath/EyeOnYouDepthCam/doc/NiTE-test/Samples/UserViewer```, you can change it at variable ```$SrcPath```in Line 10.

Default Include path in repo is ```$RepoPath/EyeOnYouDepthCam/doc/NiTE-test/``` Include, you can change it at variable ```$IncludePath``` in Line 11.

Default OpenNI library path in repo is ```$RepoPath/EyeOnYouDepthCam/doc/OpenNI-Linux-x64-2.2/Redist```, you can change it at variable ```$OpenNI_src``` in Line 12.

Default NiTE library path in repo is ```$RepoPath/EyeOnYouDepthCam/doc/NiTE-Linux-x64-2.2/Redist```, you can change it at variable $NiTE_src in Line 13.

**Description :**

This script would auto copy src file and all necessary Include file from your repo to the ROS workspace, and auto create the **CMakeLists.txt.**
After copying, it would auto-compile the package.



