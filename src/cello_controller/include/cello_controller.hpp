#ifndef MY_MOVEIT_CONTROLLER_HPP
#define MY_MOVEIT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class Cellocontroller : public rclcpp::Node {
public:
    Cellocontroller(const rclcpp::NodeOptions& options);

private:
    void execute_trajectory();
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

#endif // MY_MOVEIT_CONTROLLER_HPP