
gnome-terminal -- bash -c "ros2 run robo_driver driver ; exec bash"

sleep 2
gnome-terminal -- bash -c "ros2 run viola_controller controller ; exec bash"

sleep 0.5
gnome-terminal -- bash -c "ros2 launch viola_configure actual_robot_demo.launch.py; exec bash"