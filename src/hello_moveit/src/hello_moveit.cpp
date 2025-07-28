#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
 
using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
  // 初始化ROS并新建节点
  rclcpp::init(argc, argv);



  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  // 读取参数
  std::vector<double> max_vel = node->get_parameter("xyz").as_double_array();
  // 创建ROS日志记录器
  auto const logger = rclcpp::get_logger("hello_moveit xyz");
  RCLCPP_INFO(logger,"hello_moveit=%f,%f,%f", max_vel[0], max_vel[1], max_vel[2]);
 

  auto move_group_interface = MoveGroupInterface(node, "arm");
  move_group_interface.setPlanningTime(10.0); // 设置为 10 秒
  move_group_interface.setMaxVelocityScalingFactor(1);
  move_group_interface.setMaxAccelerationScalingFactor(0);
  // 这里定义了一个lambda表达式来创建目标姿态，是一个geometry_msgs::msg::Pose类型的消息。这个姿态被设置为x=0.28，y=-0.2，z=0.5，w=1.0 
  auto const target_pose = [max_vel]{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = sqrt(2)/2;
    msg.orientation.z = 0;
    msg.orientation.y = sqrt(2)/2;
    msg.orientation.x = 0;

    msg.position.x = max_vel[0];
    msg.position.y = max_vel[1];
    msg.position.z = max_vel[2];
    return msg;
  }();



  // 随后调用move_group_interface对象中的setPoseTarget方法将这个目标姿态设置为机械臂的运动目标
  move_group_interface.setPoseTarget(target_pose);
 
  //lambda表达式中生成计划并传入msg，返回到列表中plan
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
 
  // 判断标志位，执行plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  // 关闭ROS 
  rclcpp::shutdown();
  return 0;
}