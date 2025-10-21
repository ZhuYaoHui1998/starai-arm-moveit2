#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "robo_interfaces/msg/position_orientation.hpp"
#include "robo_interfaces/msg/gripper_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

// 定义全局变量
std::shared_ptr<rclcpp::Publisher<robo_interfaces::msg::GripperCommand>> gripper_publisher;
std::shared_ptr<rclcpp::Node> global_node;

// 存储关节状态信息的全局变量
double g_joint7_left_radians = 0.0;
double g_joint7_left_degrees = 0.0;
std::string g_gripper_state = "unknown";

// 处理joint_states消息的回调函数
void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 查找joint7_left关节的索引
  int joint7_left_index = -1;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "joint7_left") {
      joint7_left_index = i;
      break;
    }
  }

  // 如果找到joint7_left关节
  if (joint7_left_index != -1) {
    // 获取关节弧度值
    double radians = msg->position[joint7_left_index];
    // 转换为角度 (弧度 * 180 / PI)
    double degrees = ((radians / 0.032) * 100) + 100;
    
    // 创建GripperCommand消息
    auto gripper_msg = robo_interfaces::msg::GripperCommand();
    
    // 判断角度，设置命令
    if (degrees >= 90) {  // 如果角度大于等于90度，认为是open状态
      gripper_msg.command = "open";
    } else {  // 否则认为是close状态
      gripper_msg.command = "close";
    }
    
    // 发布消息
    gripper_publisher->publish(gripper_msg);
    
    // 更新全局变量，以便在main函数中统一打印
    g_joint7_left_radians = radians;
    g_joint7_left_degrees = degrees;
    g_gripper_state = gripper_msg.command;
  }
}

int main(int argc, char * argv[])
{
  // 初始化ROS并新建节点
  rclcpp::init(argc, argv);

  global_node = std::make_shared<rclcpp::Node>("arm_moveit_read");

  // 创建发布者
  auto publisher = global_node->create_publisher<robo_interfaces::msg::PositionOrientation>(
    "real_position_orientation", 10);
    
  // 创建GripperCommand消息的发布者
  gripper_publisher = global_node->create_publisher<robo_interfaces::msg::GripperCommand>(
    "real_gripper_command", 10);
    
  // 创建joint_states话题的订阅者
  auto joint_states_subscriber = global_node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, joint_states_callback);

  // 使用异步执行器来处理ROS消息回调
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(global_node);

  // 启动异步执行器线程
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  // 打印启动日志
  RCLCPP_INFO(global_node->get_logger(), "ArmMoveitRead node started");

  // 初始化 MoveGroupInterface
  auto move_group_interface = MoveGroupInterface(global_node, "arm");
  move_group_interface.allowReplanning(true);

  // 主循环
  while (rclcpp::ok())
  {
    // 设置循环频率为1Hz
    rclcpp::Rate rate(100);
    
    // 获取当前位姿
    auto get_pose = move_group_interface.getCurrentPose();

    // 创建消息并填充数据
    auto message = robo_interfaces::msg::PositionOrientation();
    message.position_x = get_pose.pose.position.x;
    message.position_y = get_pose.pose.position.y;
    message.position_z = get_pose.pose.position.z;
    message.orientation_x = get_pose.pose.orientation.x;
    message.orientation_y = get_pose.pose.orientation.y;
    message.orientation_z = get_pose.pose.orientation.z;
    message.orientation_w = get_pose.pose.orientation.w;

    // 发布消息
    publisher->publish(message);

    // 打印合并的日志信息
    // RCLCPP_INFO(global_node->get_logger(),
    //             "position = x:%.3f, y:%.3f, z:%.3f / orientation = x:%.3f, y:%.3f, z:%.3f, w:%.3f / Joint7_left: %.2f radians (%.2f degrees) - Gripper: %s",
    //             get_pose.pose.position.x, get_pose.pose.position.y, get_pose.pose.position.z,
    //             get_pose.pose.orientation.x, get_pose.pose.orientation.y,
    //             get_pose.pose.orientation.z, get_pose.pose.orientation.w,
    //             g_joint7_left_radians, g_joint7_left_degrees, g_gripper_state.c_str());
    
    // 按照设定的频率休眠
    rate.sleep();
  }

  // 停止异步执行器
  executor.cancel(); // 取消所有待处理的回调
  executor_thread.join();  // 等待异步线程结束

  // 关闭ROS 
  rclcpp::shutdown();
  return 0;
}