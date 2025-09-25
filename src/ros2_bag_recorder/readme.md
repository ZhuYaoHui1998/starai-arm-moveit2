ros2 run robo_driver driver --ros-args -p lock:='disable'
ros2 run ros2_bag_recorder bag_recorder 
ros2 bag play ./bag/my_bag
