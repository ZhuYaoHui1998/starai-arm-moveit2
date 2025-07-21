# Starai Arm 机械臂ROS2 Moveit 使用教程/Starai Arm Manipulator - ROS2 MoveIt Guide

<div align="center">
  <div style="display: flex; gap: 1rem; justify-content: center; align-items: center;" >
    <img
      src="src\viola_description\images\viola_and_violin.jpg"
      alt="SO-101 follower arm"
      title="SO-101 follower arm"
      style="width: 40%;"
    />
    <img
      src="src\viola_description\images\cello.jpg"
      alt="SO-101 leader arm"
      title="SO-101 leader arm"
      style="width: 40%;"
    />
  </div>

## 环境依赖/Dependent Environment

No LSB modules are available.

Distributor ID: Ubuntu

Description:    Ubuntu 22.04.5 LTS

Release:        22.04

Codename:       jammy

ROS2:           humble

### 安装ROS2 Humble/Install ROS2 Humble

(ROS2 Humble 安装指南)[https://wiki.seeedstudio.com/cn/install_ros2_humble/]
(ROS2 Humble Installation)(https://wiki.seeedstudio.com/install_ros2_humble/)


### 安装moveit2

安装moveit2

```sh
sudo apt install ros-humble-moveit*
```

### 安装舵机SDK库

```sh
sudo pip install pyserial
sudo pip install fashionstar-uart-sdk
```



## 使用虚拟机械臂

```sh
ros2 launch viola_configure demo.launch.py 
```




## 使用真实的机械臂

### 开启手臂控制节点

手臂会移动到零位

```sh
ros2 run robo_driver driver
```

### 开启控制器节点

```sh
ros2 run viola_controller controller
```

### 启动moveit2

```sh
ros2 launch viola_configure actual_robot_demo.launch.py
```




## FAQ

如果rivz2界面出现频闪，可以尝试以下指令。

```sh
export QT_AUTO_SCREEN_SCALE_FACTOR=0
```

