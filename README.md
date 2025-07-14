# starai-arm-moveit2

## 环境依赖

No LSB modules are available.

Distributor ID: Ubuntu

Description:    Ubuntu 22.04.5 LTS

Release:        22.04

Codename:       jammy

ROS2:           humble


## 使用虚拟机械臂
`ros2 launch viola_configure demo.launch.py `


## 使用真实的机械臂

### 开启手臂控制节点

手臂会移动到零位

`ros2 run robo_driver driver`

### 开启控制器节点

`ros2 run viola_controller controller`

## 启动moveit2

`ros2 launch viola_configure actual_robot_demo.launch.py `


