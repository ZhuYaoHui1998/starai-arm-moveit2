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
mkdir -p ~/starai-arm-moveit2/src
cd ~/starai-arm-moveit2
colcon build
```

### 克隆starai-arm-moveit2功能包/Clone `starai-arm-moveit2` Ros2's Package
```
cd ~/starai-arm-moveit2/src
git clone https://github.com/Welt-liu/starai-arm-moveit2.git
cd ~/starai_ws
colcon build
echo "source ~/starai_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Starai Arm Moveit2仿真脚本/Starai Arm MoveIt2 Simulation Script

```bash
ros2 launch viola_moveit_config demo.launch.py 
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

### viola:

```bash
ros2 run viola_controller controller
```

### cello:

```bash
ros2 run cello_controller controller
```

### 终端3:启动moveit2
### Terminal 3:Start the Moveit2

### viola:

```bash
ros2 launch viola_moveit_config actual_robot_demo.launch.py
```

### cello:

```bash
ros2 launch cello_moveit_config actual_robot_demo.launch.py
```



https://github.com/user-attachments/assets/33fa3722-f0d4-4521-818d-a49d7f6b4909

## arm_moveit_write demo & arm_moveit_read demo

### viola:
```bash
ros2 launch viola_moveit_config arm_moveit_write.launch.py
ros2 launch viola_moveit_config arm_moveit_read.launch.py
```
### cello:
```bash
ros2 launch cello_moveit_config arm_moveit_write.launch.py
ros2 launch cello_moveit_config arm_moveit_read.launch.py
```

### 位姿话题发送节点demo:

```bash
ros2 run arm_moveit_write topic_publisher
```

## FAQ

- 如果rivz2界面出现频闪，可以尝试以下指令/
  If you experience flickering in the RViz2 interface, try the following commands:

    ```bash
    export QT_AUTO_SCREEN_SCALE_FACTOR=0
    ```



ros2 launch viola_moveit_config driver.launch.py
ros2 launch viola_moveit_config actual_robot_demo.launch.py
ros2 launch viola_moveit_config moveit_write_read.py
ros2 run arm_moveit_write topic_publisher 

ros2 launch cello_moveit_config driver.launch.py
ros2 launch cello_moveit_config actual_robot_demo.launch.py
ros2 launch cello_moveit_config moveit_write_read.py
ros2 run arm_moveit_write topic_publisher 