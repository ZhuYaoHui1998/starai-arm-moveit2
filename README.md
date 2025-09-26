# Starai Arm 机械臂-ROS2 Moveit 使用教程 Starai Arm Manipulator - ROS2 MoveIt Guide

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

### 安装舵机SDK库/Install servo's SDK

```bash
sudo pip install pyserial
sudo pip install fashionstar-uart-sdk
```

### 克隆star-arm-moveit2功能包/Clone `star-arm-moveit2` Ros2's Package

```bash
cd ~/
git clone https://github.com/Welt-liu/star-arm-moveit2.git
cd ~/star-arm-moveit2
colcon build
echo "source ~/star-arm-moveit2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

https://github.com/user-attachments/assets/33fa3722-f0d4-4521-818d-a49d7f6b4909

## viola

### 激活机械臂&MoveIt2/Activate the robotic arm & MoveIt2

#### 使用虚拟机械臂/Using a virtual robotic arm

```bash
ros2 launch viola_moveit_config demo.launch.py 
```

#### 使用真实的机械臂/Using a real robotic arm

终端1:启动手臂硬件驱动/Terminal 1: Start the arm hardware driver

手臂会移动到零位/The Arm Will Move to The Zero Position.

```bash
ros2 launch viola_moveit_config driver.launch.py
```

终端2:启动moveit2/Terminal 2:Starthe Moveit2

```bash
ros2 launch viola_moveit_config actual_robot_demo.launch.py
```

到此可实现虚拟机械臂控制真实机械臂的功能/At this point, you can control the real robotic arm with the virtual robotic arm.

#### 手臂末端位姿读写示例/End-effector pose read/write demo

终端3：启动末端位姿读写示例/Terminal 3: Start the end-effector pose read/write demo

```bash
ros2 launch viola_moveit_config moveit_write_read.launch.py
```

#### 位姿话题发送节点示例/Position and orientation topic sending node demo

请更新文件/update here

src/arm_moveit_write/src/topic_publisher.cpp

```cpp
    // viola
    dataset1_ = { 
    {0.003, -0.204, 0.274},       // position
    {0.014, 0.717, 0.017, 0.696}, // orientation
       "open"                         // gripper_state
     };
     dataset2_ = {
       {-0.00, -0.34, 0.177},        // position
       {0.0, 0.7071, 0.0, 0.7071},   // orientation
       "close"                        // gripper_state
     };

    // cello
    //dataset1_ = {
    //  {-0.278, 0.000, 0.438},       // position
    //  {0.707, 0.000, -0.707, 0.000}, // orientation
    //  "open"                         // gripper_state
    //};
    //dataset2_ = {
    //  {-0.479, -0.000, 0.369},        // position
    //  {0.707, -0.000, -0.707, 0.000},   // orientation
    //  "close"                        // gripper_state
    //}

```

终端4：启动位姿话题发送节点/Terminal 4: Start the position and orientation topic sending node

```bash
colcon build
source install/setup.sh
ros2 run arm_moveit_write topic_publisher 
```

### MoveIt2-gazebo仿真机械臂例程/MoveIt2-Gazebo Simulation Robot Arm Example

> [!TIP]
>
> 在关闭gazebo图形界面后，建议在终端使用 `pkill -9 -f gazebo` 命令彻底关闭
> 在运行例程前，需要关闭其他所有正在运行的节点。

1. 安装gazebo/Install gazebo

   ```bash
   sudo apt install gazebo
   sudo apt install ros-humble-moveit*
   ```

终端1:启动gazebo图形界面/Terminal 1: Launch the Gazebo graphical user interface

```bash
ros2 launch viola_gazebo viola_gazebo.launch.py
```

终端2:启动moveit2界面/Terminal 2:Launch the MoveIt2 interface

```bash
ros2 launch viola_moveit_config gazebo_demo.launch.py
```

### 机械臂示教模式/Teaching Mode for the Robotic Arm

> [!TIP]
> 需要重新录制轨迹，请将bag文件夹删除/You need to record a new trajectory, please delete the bag folder.

终端1:启动手臂硬件驱动(示教模式)/Terminal 1: Start the arm hardware driver (teaching mode)

```bash
ros2 run robo_driver driver --ros-args -p lock:='disable'
```

终端2:记录手臂轨迹/Terminal 2: Record arm trajectory

按下回车开始录制，再按下回车结束录制，通过dataset参数指定保存路径/Press Enter to start recording, press Enter to end recording, and specify the save path through the dataset parameter.

```bash
ros2 run ros2_bag_recorder bag_recorder --ros-args -p dataset:=star/record-test
```

终端3:重播运行轨迹/Terminal 3: Replay the recorded trajectory

```bash
ros2 bag play ./star/record-test
```

## cello

### 激活机械臂&MoveIt2/Activate the robotic arm & MoveIt2

#### 使用虚拟机械臂/Using a virtual robotic arm

```bash
ros2 launch cello_moveit_config demo.launch.py 
```

#### 使用真实的机械臂/Using a real robotic arm

终端1:启动手臂硬件驱动/Terminal 1: Start the arm hardware driver

手臂会移动到零位/The Arm Will Move to The Zero Position.

```bash
ros2 launch cello_moveit_config driver.launch.py
```

终端2:启动moveit2/Terminal 2:Starthe Moveit2

```bash
ros2 launch cello_moveit_config actual_robot_demo.launch.py
```

到此可实现虚拟机械臂控制真实机械臂的功能/At this point, you can control the real robotic arm with the virtual robotic arm.

#### 手臂末端位姿读写示例/End-effector pose read/write demo

终端3：启动末端位姿读写示例/Terminal 3: Start the end-effector pose read/write demo

```bash
ros2 launch cello_moveit_config moveit_write_read.launch.py
```

#### 位姿话题发送节点示例/Position and orientation topic sending node demo

请更新文件/update here

src/arm_moveit_write/src/topic_publisher.cpp

```cpp
    // // viola
    // dataset1_ = { 
    //   {0.003, -0.204, 0.274},       // position
    //   {0.014, 0.717, 0.017, 0.696}, // orientation
    //   "open"                         // gripper_state
    // };
    // dataset2_ = {
    //   {-0.00, -0.34, 0.177},        // position
    //   {0.0, 0.7071, 0.0, 0.7071},   // orientation
    //   "close"                        // gripper_state
    // };

    // cello
    dataset1_ = {
      {-0.278, 0.000, 0.438},       // position
      {0.707, 0.000, -0.707, 0.000}, // orientation
      "open"                         // gripper_state
    };
    dataset2_ = {
      {-0.479, -0.000, 0.369},        // position
      {0.707, -0.000, -0.707, 0.000},   // orientation
      "close"                        // gripper_state
    }

```

终端4：启动位姿话题发送节点/Terminal 4: Start the position and orientation topic sending node

```bash
colcon build
source install/setup.sh
ros2 run arm_moveit_write topic_publisher 
```

### MoveIt2-gazebo仿真机械臂例程/MoveIt2-Gazebo Simulation Robot Arm Example

> [!TIP]
> 在运行例程前，需要关闭其他所有正在运行的节点。
> 在关闭gazebo图形界面后，建议在终端使用 `pkill -9 -f gazebo` 命令彻底关闭

1. 安装gazebo/Install gazebo

   ```bash
   sudo apt install gazebo
   sudo apt install ros-humble-moveit*
   ```

终端1:启动gazebo图形界面/Terminal 1: Launch the Gazebo graphical user interface

```bash
ros2 launch cello_gazebo cello_gazebo.launch.py
```

终端2:启动moveit2界面/Terminal 2:Launch the MoveIt2 interface

```bash
ros2 launch cello_moveit_config gazebo_demo.launch.py
```

### 机械臂示教模式/Teaching Mode for the Robotic Arm

> [!TIP]
> 需要重新录制轨迹，请将bag文件夹删除/You need to record a new trajectory, please delete the bag folder.

终端1:启动手臂硬件驱动(示教模式)/Terminal 1: Start the arm hardware driver (teaching mode)

```bash
ros2 run robo_driver driver --ros-args -p lock:='disable'
```

终端2:记录手臂轨迹/Terminal 2: Record arm trajectory

按下回车开始录制，再按下回车结束录制，通过dataset参数指定保存路径/Press Enter to start recording, press Enter to end recording, and specify the save path through the dataset parameter.

```bash
ros2 run ros2_bag_recorder bag_recorder --ros-args -p dataset:=star/record-test
```

终端3:重播运行轨迹/Terminal 3: Replay the recorded trajectory

```bash
ros2 bag play ./star/record-test
```

## FAQ

- 如果rivz2界面出现频闪，可以尝试以下指令/
  If you experience flickering in the RViz2 interface, try the following commands:

    ```bash
    export QT_AUTO_SCREEN_SCALE_FACTOR=0
    ```
