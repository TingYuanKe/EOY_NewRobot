# Robot Control API

## API Documention
### Include
```cpp
#include "robot_control_api.h"
```

### Initialization
- void rosInit(int argc, char **argv, const char *node_name);
    - 在main函式call一次即可

### API
#### Motion Control
- void goForward();

- void goBack();

- void spinLeft();

- void spinRight();

- void turnLeftRight(int left_speed, int right_speed);
    - 速度限制: 350 ~ 30000
    - 左轉: left_speed < right_speed
    - 右轉: left_speed > right_speed

- void stopMotion();

- 要call goForward(), goBack() 前請先 call setSpeed() 將左右輪速度調成一致

#### Speed Control
- void resetSpeed();
    - 將速度reset為350

- void setSpeed(int left_speed, int right_speed);
    - 速度限制: 350 ~ 30000

- void setAcceleration(int left_acc, int right_acc)
    - 設定左右輪的加/減速度
    - 預設值: 300

#### Arm Angle Control
- void armUp();
    - 前後臂抬起至90度

- void armDown();
    - 前後臂平放

## Compile Guide
### 創建ros project
1. cd ~/catkin_src

2. catkin_create_pkg <package_name> std_msgs rospy roscpp
    - e.g. catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

3. source code 放在 ~/catkin/src/<package_name>/src 中

### 編譯步驟
0. code 放在`~/catkin_ws/src/<package_name>/src`

1. `cd ~/catkin_ws/src/<package_name>`

2. 改 CMakeLists.txt
    - `add_executable(<node_name>, src/<your_code>.cpp)`
    - `target_link_libraries(<node_name> ${catkin_LIBRARIES})`

3. `cd ~/catkin_make`

4. 編譯成功後, `source ~/catkin_ws/devel/setup.bash`

5. 執行︰`rosrun <package_name> <node_name> (須先執行roscore)`
