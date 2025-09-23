#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"  // 添加Float32MultiArray头文件
#include "sensor_msgs/msg/joint_state.hpp"
#define LEADER_ARM_ANGLE_TOPIC "joint_states"
#include "robo_interfaces/msg/set_angle.hpp"
#include <string>

std::string joint_name[] = {
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7"
};

float jointstate2servoangle(uint8_t servo_id,float joint_state)
{
    if (servo_id < 6)
        return joint_state * (180 / 3.1415926);
    else if(servo_id == 6)
        return (joint_state / 0.032) * 100 + 100;
}



class BagRecorder : public rclcpp::Node {
public:
  BagRecorder() : Node("bag_recorder") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    // writer_->open("my_bag");
    // writer_->open("my_bag");
    writer_->open("bag/my_bag");

    // 修改为订阅Float32MultiArray话题
    angle_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      LEADER_ARM_ANGLE_TOPIC, 10,
      [this](const sensor_msgs::msg::JointState& msg) {

        writer_->write(msg, LEADER_ARM_ANGLE_TOPIC, this->now());

        auto custom_msg = std::make_unique<robo_interfaces::msg::SetAngle>();
        
        // 2. 填充自定义消息字段（根据你的需求映射）
        custom_msg->header = msg.header;  // 保留时间戳和坐标系
        custom_msg->joint_names = msg.name;  // 关节名称
        custom_msg->positions = msg.position;  // 关节位置
        for(int i=0;i<7)
        {


        }
          custom_msg->servo_id = [0,1,2,3,4,5,6];
          custom_msg->target_angle = [
          jointstate2servoangle(0,msg.position[0]),
          jointstate2servoangle(1,msg.position[1]),
          jointstate2servoangle(2,msg.position[2]),
          jointstate2servoangle(3,msg.position[3]),
          jointstate2servoangle(4,msg.position[4]),
          jointstate2servoangle(5,msg.position[5]),
          jointstate2servoangle(6,msg.position[6])
        ];


        
        // 3. 记录自定义消息到bag
        writer_->write(
            *custom_msg, 
            "custom_joint_states",  // 自定义话题名
            custom_msg->header.stamp  // 使用原始时间戳
        );


      });

    RCLCPP_INFO(get_logger(), "Bag recorder started!");
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr angle_sub_;  // 修改订阅器类型
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagRecorder>());
  rclcpp::shutdown();
  return 0;
}
