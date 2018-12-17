## Auto create ROS workspace and package
#### ROSws.sh<span></span>
**Usage :**
```
./ROSws.sh <package_name>
```
package_name : the name of the ROS package.

Default path of workspace is at your $HOME, you can change it at variable $path.

**Description :** 

It would auto-create the catkin_ws, before creating a new workspace, it could remove the existed one.
After creating package, it would copy the original CMakeLists.txt as CMakeLists.txt.orig to use for auto-create CMakeLists.txt in ROSmake.sh<span></span>.

## Auto copy src file and auto compile
#### ROSmake_EOY.sh<span></span>
**Usage :**
```
./ROSmake_EOY.sh <EOY_Repo_path> <package_name>"
```

EOY_Repo_path: The path of the EOY repo which is download from github.
package_name : The name of the ROS package.

Default path of workspace is at your ```$HOME/catkin_ws```, you can change it at variable ```$workspace``` in Line 17.

Default source code path in repo is ```$RepoPath/EyeOnYouDepthCam/exec/src```, you can change it at variable ```$SrcPath```in Line 10.

Default Include path in repo is ```$RepoPath/EyeOnYouDepthCam/Include``` Include, you can change it at variable ```$IncludePath``` in Line 11.

Default OpenNI library path in repo is ```$RepoPath/EyeOnYouDepthCam/lib/OpenNI-Linux-x64-2.2/Redist```, you can change it at variable ```$OpenNI_src``` in Line 12.

Default NiTE library path in repo is ```$RepoPath/EyeOnYouDepthCam/lib/NiTE-Linux-x64-2.2/Redist```, you can change it at variable $NiTE_src in Line 13.

**Description :**

This script would auto copy EOY src file and all necessary Include file from your repo to the ROS workspace, and auto create the **CMakeLists.txt.**
After copying, it could be chosen to auto-compile the package.


#### ROSmake_Manual.sh<span></span>
**Usage :**
```
./ROSmake_Manual.sh <EOY_Repo_path> <package_name>"
```

EOY_Repo_path: The path of the EOY repo which is download from github.
package_name : The name of the ROS package.

Default path of workspace is at your ```$HOME/catkin_ws```, you can change it at variable ```$workspace```.

Default source code path in repo is ```$RepoPath/RobotRemoteControl```, you can change it at variable ```$SrcPath```.


**Description :**

This script would auto copy ROS manual_control src file and all necessary Include file from your repo to the ROS workspace, and auto create the **CMakeLists.txt.**
After copying, it could be chosen to auto-compile the package.





