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
      {0.35, -0.00, 0.23},       // position
      {-0.499, 0.500, -0.500, 0.500}, // orientation
      "open"                         // gripper_state
    };
    dataset2_ = {
      {0.15, -0.00, 0.299},        // position
      {0.500, -0.500, 0.500, -0.499},   // orientation
      "close"                        // gripper_state
    };

    //cello
    // dataset1_ = {
    //   {0.278, 0.000, 0.438},       // position
    //   {-0.506, 0.507, -0.496, 0.491}, // orientation
    //   "open"                         // gripper_state
    // };
    // dataset2_ = {
    //   {0.479, -0.000, 0.369},        // position
    //   {-0.506, 0.507, -0.496, 0.491}, // orientation
    //   "close"                        // gripper_state
    // };

    timer_ = this->create_wall_timer(
      3000ms, std::bind(&CombinedPublisher::timer_callback, this));
  }

  ~CombinedPublisher() {
    if (timer_) {
      timer_->cancel();
    }
    RCLCPP_INFO(this->get_logger(), "CombinedPublisher shutting down, timer canceled.");
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
