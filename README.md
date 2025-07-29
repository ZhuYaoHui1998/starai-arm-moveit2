# Starai Arm 机械臂ROS2 Moveit 使用教程
# Starai Arm Manipulator - ROS2 MoveIt Guide

<div align="center">
  <div style="display: flex; gap: 1rem; justify-content: center; align-items: center;" >
    <img
      src="src\viola_description\images\viola_and_violin.jpg"
      alt="SO-101 follower arm"
      title="SO-101 follower arm"
      style="width: 80%;"
    />
    <img
      src="src\viola_description\images\cello.jpg"
      alt="SO-101 leader arm"
      title="SO-101 leader arm"
      style="width: 80%;"
    />
  </div>
</div>

## 环境依赖/Dependent Environment

No LSB modules are available.

Distributor ID: Ubuntu

Description:    Ubuntu 22.04.5 LTS

Release:        22.04

Codename:       Jammy

ROS2:           Humble

### 安装ROS2 Humble/Install ROS2 Humble

[ROS2 Humble 安装指南](https://wiki.seeedstudio.com/cn/install_ros2_humble/)

[ROS2 Humble Installation](https://wiki.seeedstudio.com/install_ros2_humble/)


### 安装Moveit2/Install Moveit2

```bash
sudo apt install ros-humble-moveit*
```

### 安装舵机SDK库/Install Servo Motor's SDK

```bash
sudo pip install pyserial
sudo pip install fashionstar-uart-sdk
```

### 创建工作空间并初始化/Create a workspace and Initialization.

```bash
mkdir -p ~/starai_ws/src
cd ~/starai_ws
colcon build
```

### 克隆starai-arm-moveit2功能包/Clone `starai-arm-moveit2` Ros2's Package
```
cd ~/starai_ws/src
git clone https://github.com/Welt-liu/starai-arm-moveit2.git
cd ~/starai_ws
colcon build
echo "source ~/starai_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Starai Arm Moveit2仿真脚本/Starai Arm MoveIt2 Simulation Script

```bash
ros2 launch viola_configure demo.launch.py 
```


## 使用真实的机械臂/Using a Real Robotic Arm

### 终端1:开启手臂控制节点
### Terminal 1: Start the Arm Control Node

手臂会移动到零位/The Arm Will Move to The Zero Position.

```bash
ros2 run robo_driver driver
```

### 终端2:开启控制器节点
### Terminal 2:Start the Controller Node

```bash
ros2 run viola_controller controller
```

### 终端3:启动moveit2
### Terminal 3:Start the Moveit2

```bash
ros2 launch viola_configure actual_robot_demo.launch.py
```



https://github.com/user-attachments/assets/33fa3722-f0d4-4521-818d-a49d7f6b4909



## FAQ

- 如果rivz2界面出现频闪，可以尝试以下指令/
If you experience flickering in the RViz2 interface, try the following commands:

    ```bash
    export QT_AUTO_SCREEN_SCALE_FACTOR=0
    ```



## test

ros2 run hello_moveit hello_moveit --ros-args -p xyz:="[-0.0003, -0.2, 0.09]"

ros2 launch viola_moveit_config hello_moveit.launch.py 