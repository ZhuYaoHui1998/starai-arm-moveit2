#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"  // 添加Float32MultiArray头文件

#define LEADER_ARM_ANGLE_TOPIC "leader_arm_angle_topic0"

class BagRecorder : public rclcpp::Node {
public:
  BagRecorder() : Node("bag_recorder") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    // writer_->open("my_bag");
    // writer_->open("my_bag");
    writer_->open("bag/my_bag");

    // 修改为订阅Float32MultiArray话题
    angle_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      LEADER_ARM_ANGLE_TOPIC, 10,
      [this](const std_msgs::msg::Float32MultiArray& msg) {
        // 注意：Float32MultiArray没有header.stamp，使用当前时间
        writer_->write(msg, LEADER_ARM_ANGLE_TOPIC, this->now());
        RCLCPP_INFO(get_logger(), "Recorded arm angles: %zu values", msg.data.size());
      });

    RCLCPP_INFO(get_logger(), "Bag recorder started!");
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr angle_sub_;  // 修改订阅器类型
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagRecorder>());
  rclcpp::shutdown();
  return 0;
}
