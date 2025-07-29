#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "hello_moveit.hpp"
using moveit::planning_interface::MoveGroupInterface;



int main(int argc, char * argv[])
{
  // 初始化ROS并新建节点
  rclcpp::init(argc, argv);



  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

    // auto node_parameter = rclcpp::Node::make_shared("arm_params", node);
        // 在构造函数或初始化函数中添加

    double msg_value[7];
    msg_value[POSITION_X] = node->get_parameter("position_x").as_double();
    node->get_parameter("position_y", msg_value[POSITION_Y]);
    node->get_parameter("position_z", msg_value[POSITION_Z]);
    node->get_parameter("orientation_x", msg_value[ORIENTATION_X]);
    node->get_parameter("orientation_y", msg_value[ORIENTATION_Y]);
    node->get_parameter("orientation_z", msg_value[ORIENTATION_Z]);
    node->get_parameter("orientation_w", msg_value[ORIENTATION_W]);
  
  
  // 创建ROS日志记录器
  auto const logger = rclcpp::get_logger("hello_moveit xyz");
  // RCLCPP_INFO(logger,"hello_moveit=%f,%f,%f", max_vel[0], max_vel[1], max_vel[2]);
  // RCLCPP_INFO(logger,"hello_moveit=%f,%f,%f,%f,%f,%f", msg_value[POSITION_X], msg_value[POSITION_Y], 
  //                                                msg_value[POSITION_Z], msg_value[ORIENTATION_W],
  //                                                msg_value[ORIENTATION_X],msg_value[ORIENTATION_Y]);

  auto move_group_interface = MoveGroupInterface(node, "arm");
  // move_group_interface.startStateMonitor();

  move_group_interface.setPlanningTime(10.0); // 设置为 10 秒
  move_group_interface.setMaxVelocityScalingFactor(1);
  move_group_interface.setMaxAccelerationScalingFactor(0);
  // 这里定义了一个lambda表达式来创建目标姿态，是一个geometry_msgs::msg::Pose类型的消息。这个姿态被设置为x=0.28，y=-0.2，z=0.5，w=1.0 
  auto const target_pose = [msg_value]{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = msg_value[ORIENTATION_W];
    msg.orientation.z = msg_value[ORIENTATION_Z];
    msg.orientation.y = msg_value[ORIENTATION_Y];
    msg.orientation.x = msg_value[ORIENTATION_X];

    msg.position.x = msg_value[POSITION_X];
    msg.position.y = msg_value[POSITION_Y];
    msg.position.z = msg_value[POSITION_Z];
    return msg;
  }();

 // 使用异步执行器来处理ROS消息回调
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // 启动异步执行器线程
  std::thread executor_thread([&]() {
    executor.spin();
  });


  // 随后调用move_group_interface对象中的setPoseTarget方法将这个目标姿态设置为机械臂的运动目标
  move_group_interface.setPoseTarget(target_pose);
 
  //lambda表达式中生成计划并传入msg，返回到列表中plan
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
 

  auto pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger,"hello_moveit=%f,%f,%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);


  // 判断标志位，执行plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  // 停止异步执行器
  executor.spin_some();  // 处理当前队列中的所有回调
  executor_thread.join();  // 等待异步线程结束

  // 关闭ROS 
  rclcpp::shutdown();
  return 0;
}