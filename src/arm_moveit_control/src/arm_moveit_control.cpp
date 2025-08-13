#include <memory>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "arm_moveit_control.hpp"
#include "robo_interfaces/msg/position_orientation.hpp"
#include "robo_interfaces/msg/set_angle.hpp"
#include "robo_interfaces/msg/gripper_command.hpp"
using moveit::planning_interface::MoveGroupInterface;



class ArmMoveitControl : public rclcpp::Node
{
public:
  ArmMoveitControl() : Node("arm_moveit_control"), logger_(this->get_logger()), is_first_message_(true), last_gripper_state_("")
  {
    // 创建订阅者，订阅位置和方向信息
    subscription_ = this->create_subscription<robo_interfaces::msg::PositionOrientation>(
      "position_orientation_topic", 10,
      std::bind(&ArmMoveitControl::topic_callback, this, std::placeholders::_1));
    
    // 创建订阅者，订阅夹爪控制信息
    gripper_subscription_ = this->create_subscription<robo_interfaces::msg::GripperCommand>(
      "gripper_command_topic", 10,
      std::bind(&ArmMoveitControl::gripper_callback, this, std::placeholders::_1));
    
    // 创建发布者，用于发送夹爪控制命令
    set_angle_publisher_ = this->create_publisher<robo_interfaces::msg::SetAngle>(
      "set_angle_topic", 10);
    
    // 输出启动信息
    RCLCPP_INFO(logger_, "ARMMoveItControl node started, waiting for position and orientation messages...");
    
    // 初始化MoveGroupInterface
    move_group_interface_ = std::make_shared<MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](auto) {}), "arm");
    
    // 设置运动规划参数
    move_group_interface_->setPlanningTime(10.0);               // 设置为 10 秒
    move_group_interface_->setMaxVelocityScalingFactor(1);
    move_group_interface_->setMaxAccelerationScalingFactor(0);
    move_group_interface_->allowReplanning(true);              // 允许重规划
    move_group_interface_->setNumPlanningAttempts(10);        // 增加尝试次数
    
    // 初始化上一次的位置和方向
    last_position_x_ = 0.0;
    last_position_y_ = 0.0;
    last_position_z_ = 0.0;
    last_orientation_x_ = 0.0;
    last_orientation_y_ = 0.0;
    last_orientation_z_ = 0.0;
    last_orientation_w_ = 0.0;
  }

private:
  void topic_callback(const robo_interfaces::msg::PositionOrientation::SharedPtr msg)
  {
    // 检查是否与上一次的位置和方向相同
    const double position_tolerance = 0.001;  // 位置容差
    const double orientation_tolerance = 0.001;  // 方向容差
    
    bool is_same_position = 
      std::abs(msg->position_x - last_position_x_) < position_tolerance &&
      std::abs(msg->position_y - last_position_y_) < position_tolerance &&
      std::abs(msg->position_z - last_position_z_) < position_tolerance;
    
    bool is_same_orientation = 
      std::abs(msg->orientation_x - last_orientation_x_) < orientation_tolerance &&
      std::abs(msg->orientation_y - last_orientation_y_) < orientation_tolerance &&
      std::abs(msg->orientation_z - last_orientation_z_) < orientation_tolerance &&
      std::abs(msg->orientation_w - last_orientation_w_) < orientation_tolerance;
    
    // 如果是第一条消息或者位置/方向发生了变化，则执行移动
    if (is_first_message_ || !(is_same_position && is_same_orientation)) {      
      // 创建目标姿态
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = msg->position_x;
      target_pose.position.y = msg->position_y;
      target_pose.position.z = msg->position_z;
      target_pose.orientation.x = msg->orientation_x;
      target_pose.orientation.y = msg->orientation_y;
      target_pose.orientation.z = msg->orientation_z;
      target_pose.orientation.w = msg->orientation_w;
      
      // 设置目标姿态
      move_group_interface_->setPoseTarget(target_pose);
      
      // 生成运动规划
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group_interface_->plan(plan));
      
      // 执行规划
      if(success) {
        move_group_interface_->execute(plan);
        move_group_interface_->clearPoseTargets();  // 清除目标姿态，确保下次规划可以重新开始
      } else {
        RCLCPP_ERROR(logger_, "Planning failed!");
      }
      
      // 如果是第一条消息，直接跳过后续检查
      if (is_first_message_) {
        is_first_message_ = false;
        return;
      }
    }
    
    // 更新上一次的位置和方向
    last_position_x_ = msg->position_x;
    last_position_y_ = msg->position_y;
    last_position_z_ = msg->position_z;
    last_orientation_x_ = msg->orientation_x;
    last_orientation_y_ = msg->orientation_y;
    last_orientation_z_ = msg->orientation_z;
    last_orientation_w_ = msg->orientation_w;

    RCLCPP_INFO(logger_, "Position: x=%f, y=%f, z=%f | Orientation: x=%f, y=%f, z=%f, w=%f", 
                msg->position_x, msg->position_y, msg->position_z, msg->orientation_x, msg->orientation_y, msg->orientation_z, msg->orientation_w);

  }

  // 控制夹爪函数
  void control_gripper(const std::string& gripper_state)
  {
    // 创建 SetAngle 消息
    auto msg = robo_interfaces::msg::SetAngle();
    
    // 设置舵机ID为6（夹爪）
    msg.servo_id = {6};
    
    // 根据夹爪状态设置角度
    double angle = 0.0;
    if (gripper_state == "open") {
      angle = 100.0;  // 打开夹爪的角度
    } else if (gripper_state == "close") {
      angle = 0.0;    // 关闭夹爪的角度
    }
    
    // 使用角度
    msg.target_angle = {angle};
    msg.time = {1500};  // 设置执行时间为1.5秒

    set_angle_publisher_->publish(msg);
    
  }
  // 夹爪控制回调函数
  void gripper_callback(const robo_interfaces::msg::GripperCommand::SharedPtr msg)
  {    
    // 如果夹爪状态发生变化，则控制夹爪
    // if (last_gripper_state_ != msg->command) {
    //   control_gripper(msg->command);
    //   last_gripper_state_ = msg->command;
    //   RCLCPP_INFO(logger_, "Received gripper command: %s", msg->command.c_str());
    // }

    // 记录收到的消息
    RCLCPP_INFO(logger_, "Received gripper command: %s (previous state: %s)", 
                msg->command.c_str(), last_gripper_state_.c_str());
    // 无论状态是否变化，都执行夹爪控制
    control_gripper(msg->command);
    last_gripper_state_ = msg->command;
  }

  rclcpp::Subscription<robo_interfaces::msg::PositionOrientation>::SharedPtr subscription_;
  rclcpp::Subscription<robo_interfaces::msg::GripperCommand>::SharedPtr gripper_subscription_;
  rclcpp::Publisher<robo_interfaces::msg::SetAngle>::SharedPtr set_angle_publisher_;
  std::shared_ptr<MoveGroupInterface> move_group_interface_;
  rclcpp::Logger logger_;
  
  // 用于存储上一次的位置和方向
  double last_position_x_;
  double last_position_y_;
  double last_position_z_;
  double last_orientation_x_;
  double last_orientation_y_;
  double last_orientation_z_;
  double last_orientation_w_;
  bool is_first_message_;  // 标记是否是第一条消息
  std::string last_gripper_state_;  // 存储上一次的夹爪状态
};

int main(int argc, char * argv[])
{
  // 初始化ROS
  rclcpp::init(argc, argv);
  
  // 创建并运行节点
  auto node = std::make_shared<ArmMoveitControl>();
  rclcpp::spin(node);
  
  // 关闭ROS
  rclcpp::shutdown();
  return 0;
}