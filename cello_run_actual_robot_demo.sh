
gnome-terminal -- bash -c "ros2 run robo_driver driver ; exec bash"

sleep 2
gnome-terminal -- bash -c "ros2 run cello_controller controller ; exec bash"

sleep 0.5
gnome-terminal -- bash -c "ros2 launch cello_moveit_config  actual_robot_demo.launch.py; exec bash"