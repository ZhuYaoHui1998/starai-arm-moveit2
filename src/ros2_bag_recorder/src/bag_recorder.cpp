#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "std_msgs/msg/float32_multi_array.hpp" // 添加Float32MultiArray头文件
#include "sensor_msgs/msg/joint_state.hpp"
#include "robo_interfaces/msg/set_angle.hpp"
#include <string>
#include <unordered_map>
#define LEADER_ARM_ANGLE_TOPIC "joint_states"
#define ROBO_SET_ANGLE_SUBSCRIBER "set_angle_topic" // 设置角度话题 / topic for setting angles

std::vector<std::string> joint_name = {
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7_left"};

float jointstate2servoangle(uint8_t servo_id, float joint_state)
{
  if (servo_id < 6)
    return joint_state * (180 / 3.1415926);
  else if (servo_id == 6)
    return (joint_state / 0.032) * 100 + 100;
  else
    return 0;
}

class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder() : Node("bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("bag/my_bag");

    angle_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        LEADER_ARM_ANGLE_TOPIC, 10,
        [this](const sensor_msgs::msg::JointState &msg)
        {
          auto custom_msg = std::make_unique<robo_interfaces::msg::SetAngle>();

          for (size_t i = 0; i < msg.name.size(); i++)
          {
            auto id = std::find(joint_name.begin(), joint_name.end(), msg.name[i]) - joint_name.begin();
            custom_msg->servo_id.push_back(id);
            custom_msg->target_angle.push_back(jointstate2servoangle(id, msg.position[i]));
            custom_msg->time.push_back(200);
          }

          // 3. 记录自定义消息到bag
          writer_->write(
              *custom_msg,
              ROBO_SET_ANGLE_SUBSCRIBER, // 自定义话题名
              this->now()                // 使用原始时间戳
          );
        });

    RCLCPP_INFO(get_logger(), "Bag recorder started!");
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr angle_sub_; // 修改订阅器类型
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagRecorder>());
  rclcpp::shutdown();
  return 0;
}
