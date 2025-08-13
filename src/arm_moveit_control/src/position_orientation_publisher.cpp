#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robo_interfaces/msg/position_orientation.hpp"
#include "robo_interfaces/msg/gripper_command.hpp"

using namespace std::chrono_literals;

/* 这个节点用于发布位置和方向信息到 position_orientation_topic 话题 */
class PositionOrientationPublisher : public rclcpp::Node
{
public:
  PositionOrientationPublisher()
  : Node("position_orientation_publisher")
  {
    // 创建位置方向发布者
    publisher_ = this->create_publisher<robo_interfaces::msg::PositionOrientation>(
      "position_orientation_topic", 10);
      
    // 创建夹爪控制发布者
    gripper_publisher_ = this->create_publisher<robo_interfaces::msg::GripperCommand>(
      "gripper_command_topic", 10);
    
    // 初始化位置和方向
    // position_x_ = 0.003;//0.003          -0.00 
    // position_y_ = -0.204;//-0.204        -0.36
    // position_z_ = 0.274;//0.274          0.177
    // orientation_x_ = 0.014;//0.014         0.0
    // orientation_y_ = 0.717;//0.717      0.7071
    // orientation_z_ = 0.017;//0.017         0.0
    // orientation_w_ = 0.696;//0.696      0.7071
    // gripper_state_ = "open";  // 初始化夹爪状态为打开

    position_x_ = -0.00;//0.003          -0.00 
    position_y_ = -0.36;//-0.204        -0.36
    position_z_ = 0.177;//0.274          0.177
    orientation_x_ = 0.0;//0.014         0.0
    orientation_y_ = 0.7071;//0.717      0.7071
    orientation_z_ = 0.0;//0.017         0.0
    orientation_w_ = 0.7071;//0.696      0.7071
    gripper_state_ = "close";  // 初始化夹爪状态为打开

    // 声明参数
    this->declare_parameter("position_x", position_x_);
    this->declare_parameter("position_y", position_y_);
    this->declare_parameter("position_z", position_z_);
    this->declare_parameter("orientation_x", orientation_x_);
    this->declare_parameter("orientation_y", orientation_y_);
    this->declare_parameter("orientation_z", orientation_z_);
    this->declare_parameter("orientation_w", orientation_w_);
    this->declare_parameter("gripper_state", gripper_state_);
    
    // 获取参数
    position_x_ = this->get_parameter("position_x").as_double();
    position_y_ = this->get_parameter("position_y").as_double();
    position_z_ = this->get_parameter("position_z").as_double();
    orientation_x_ = this->get_parameter("orientation_x").as_double();
    orientation_y_ = this->get_parameter("orientation_y").as_double();
    orientation_z_ = this->get_parameter("orientation_z").as_double();
    orientation_w_ = this->get_parameter("orientation_w").as_double();
    gripper_state_ = this->get_parameter("gripper_state").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Position and Orientation Publisher started");
  }

  void publish_once()
  {
    // 发布位置和方向信息
    auto pos_message = robo_interfaces::msg::PositionOrientation();
    pos_message.position_x = position_x_;
    pos_message.position_y = position_y_;
    pos_message.position_z = position_z_;
    pos_message.orientation_x = orientation_x_;
    pos_message.orientation_y = orientation_y_;
    pos_message.orientation_z = orientation_z_;
    pos_message.orientation_w = orientation_w_;
    
    publisher_->publish(pos_message);
    // 发布夹爪控制信息
    auto gripper_message = robo_interfaces::msg::GripperCommand();
    gripper_message.command = gripper_state_;
      
    gripper_publisher_->publish(gripper_message);

    RCLCPP_INFO(this->get_logger(), "Publishing position: x=%f, y=%f, z=%f | orientation: x=%f, y=%f, z=%f, w=%f | gripper: %s", 
                position_x_, position_y_, position_z_, orientation_x_, orientation_y_, orientation_z_, orientation_w_, gripper_state_.c_str());

  }
  
  rclcpp::Publisher<robo_interfaces::msg::PositionOrientation>::SharedPtr publisher_;
  rclcpp::Publisher<robo_interfaces::msg::GripperCommand>::SharedPtr gripper_publisher_;
  double position_x_, position_y_, position_z_;
  double orientation_x_, orientation_y_, orientation_z_, orientation_w_;
  std::string gripper_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionOrientationPublisher>();
  node->publish_once();
  rclcpp::shutdown();
  return 0;
}