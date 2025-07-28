#include "cello_controller.hpp"

Cellocontroller::Cellocontroller(const rclcpp::NodeOptions& options)
    : Node("cello_controller", options) {
    // 初始化 MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm_group"); // "arm_group" 是 MoveIt 配置中的规划组名称

    // 执行轨迹
    execute_trajectory();
}

void Cellocontroller::execute_trajectory() {
    // 设置目标位置（示例：关节空间目标）
    std::vector<double> joint_target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group_->setJointValueTarget(joint_target);

    // 规划轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing trajectory.");
        move_group_->execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Cellocontroller>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}