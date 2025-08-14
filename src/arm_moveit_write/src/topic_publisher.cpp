#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robo_interfaces/msg/position_orientation.hpp"
#include "robo_interfaces/msg/gripper_command.hpp"

using namespace std::chrono_literals;

class CombinedPublisher : public rclcpp::Node
{
public:
  CombinedPublisher()
  : Node("combined_publisher")
  {
    pos_pub_ = this->create_publisher<robo_interfaces::msg::PositionOrientation>(
      "position_orientation_topic", 10);
    gripper_pub_ = this->create_publisher<robo_interfaces::msg::GripperCommand>(
      "gripper_command_topic", 10);

    // viola
    dataset1_ = {
      {0.003, -0.204, 0.274},       // position
      {0.014, 0.717, 0.017, 0.696}, // orientation
      "open"                         // gripper_state
    };

    dataset2_ = {
      {-0.00, -0.36, 0.177},        // position
      {0.0, 0.7071, 0.0, 0.7071},   // orientation
      "close"                        // gripper_state
    };

    // cello
    // dataset1_ = {
    //   {-0.27880, 0.0000022075, 0.43849},       // position
    //   {0.70713, 0.000034917, -0.70709, 0.000056086}, // orientation
    //   "open"                         // gripper_state
    // };

    // dataset2_ = {
    //   {-0.47959, -0.000018764, 0.36999},        // position
    //   {0.70712, -0.00001244, -0.7071, 0.000036032},   // orientation
    //   "close"                        // gripper_state
    // };

    timer_ = this->create_wall_timer(
      3000ms, std::bind(&CombinedPublisher::timer_callback, this));
  }

private:
  struct Dataset {
    std::array<double, 3> position;
    std::array<double, 4> orientation;
    std::string gripper_state;
  };

  void timer_callback()
  {
    static bool use_dataset1 = true;
    const auto& dataset = use_dataset1 ? dataset1_ : dataset2_;

    auto pos_msg = robo_interfaces::msg::PositionOrientation();
    pos_msg.position_x = dataset.position[0];
    pos_msg.position_y = dataset.position[1];
    pos_msg.position_z = dataset.position[2];
    pos_msg.orientation_x = dataset.orientation[0];
    pos_msg.orientation_y = dataset.orientation[1];
    pos_msg.orientation_z = dataset.orientation[2];
    pos_msg.orientation_w = dataset.orientation[3];
    pos_pub_->publish(pos_msg);

    auto gripper_msg = robo_interfaces::msg::GripperCommand();
    gripper_msg.command = dataset.gripper_state;
    gripper_pub_->publish(gripper_msg);

    RCLCPP_INFO(this->get_logger(), "Published %s dataset",
                use_dataset1 ? "first" : "second");
    use_dataset1 = !use_dataset1;
  }

  rclcpp::Publisher<robo_interfaces::msg::PositionOrientation>::SharedPtr pos_pub_;
  rclcpp::Publisher<robo_interfaces::msg::GripperCommand>::SharedPtr gripper_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Dataset dataset1_;
  Dataset dataset2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CombinedPublisher>());
  rclcpp::shutdown();
  return 0;
}




// #!/bin/bash

// while true; do
//     # 发布第一组数据（初始默认值）
//     ros2 topic pub /position_orientation_topic robo_interfaces/msg/PositionOrientation "
//     {
//       position_x: 0.003,
//       position_y: -0.204,
//       position_z: 0.274,
//       orientation_x: 0.014,
//       orientation_y: 0.717,
//       orientation_z: 0.017,
//       orientation_w: 0.696
//     }" --once

//     ros2 topic pub /gripper_command_topic robo_interfaces/msg/GripperCommand "
//     {
//       command: 'open'
//     }" --once

//     echo "已发布第一组数据，等待 2 秒..."
//     sleep 2

//     # 发布第二组数据（修改后的默认值）
//     ros2 topic pub /position_orientation_topic robo_interfaces/msg/PositionOrientation "
//     {
//       position_x: -0.00,
//       position_y: -0.36,
//       position_z: 0.177,
//       orientation_x: 0.0,
//       orientation_y: 0.7071,
//       orientation_z: 0.0,
//       orientation_w: 0.7071
//     }" --once

//     ros2 topic pub /gripper_command_topic robo_interfaces/msg/GripperCommand "
//     {
//       command: 'close'
//     }" --once

//     echo "已发布第二组数据，等待 2 秒..."
//     sleep 2
// done
