
/录制动作/实时动作
    ros2 run leaderfollower_armpair demo --ros-args -p leader:='/dev/ttyUSB0' --ros-args -p follower:='/dev/ttyUSB1'
    ros2 run leaderfollower_armpair demo --ros-args -p leader:='/dev/ttyUSB1' --ros-args -p follower:='/dev/ttyUSB0'
ros2 run ros2_bag_recorder bag_recorder 

/显示模型
ros2 launch robo_770_description display_rviz.launch.py 


<!-- /同步动作/启动机械臂
ros2 run leaderfollower_armpair follower --ros-args -p PORT_NAME:='/dev/ttyUSB0'
ros2 run leaderfollower_armpair follower --ros-args -p PORT_NAME:='/dev/ttyUSB1' -->

/大手臂demo/播放动作
ros2 run leaderfollower_armpair loop_demo --ros-args -p PORT_NAME:='/dev/ttyUSB0'
ros2 run leaderfollower_armpair loop_demo --ros-args -p PORT_NAME:='/dev/ttyUSB1'
<!-- ros2 bag play /home/nyancos/robo/move_blue_block --loop -->

ros2 bag play /home/nyancos/robo/FsRobo-A1/bag/my_bag --loop

ros2 run leaderfollower_armpair leader_demo --ros-args -p leader:='/dev/ttyUSB0'


