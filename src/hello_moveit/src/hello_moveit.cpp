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
 // 使用异步执行器来处理ROS消息回调
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // 启动异步执行器线程
  std::thread executorThread([&executor]() {
    executor.spin();
  });
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
  auto const hello_moveit_logger = rclcpp::get_logger("hello_moveit");
  // RCLCPP_INFO(logger,"hello_moveit=%f,%f,%f", max_vel[0], max_vel[1], max_vel[2]);
  // RCLCPP_INFO(logger,"hello_moveit=%f,%f,%f,%f,%f,%f", msg_value[POSITION_X], msg_value[POSITION_Y], 
  //                                                msg_value[POSITION_Z], msg_value[ORIENTATION_W],
  //                                                msg_value[ORIENTATION_X],msg_value[ORIENTATION_Y]);

  auto moveGroupInterface = MoveGroupInterface(node, "arm");
  // moveGroupInterface.startStateMonitor();

  moveGroupInterface.setPlanningTime(10.0);               // 设置为 10 秒
  moveGroupInterface.setMaxVelocityScalingFactor(1);
  moveGroupInterface.setMaxAccelerationScalingFactor(0);
  // moveGroupInterface.setGoalTolerance(0.01);             // 位置容差
  // moveGroupInterface.setGoalOrientationTolerance(0.01);  // 方向容差
  moveGroupInterface.allowReplanning(true);              // 允许重规划
  moveGroupInterface.setNumPlanningAttempts(10);        // 增加尝试次数  
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

  RCLCPP_INFO(hello_moveit_logger,"target position    = %f,%f,%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
  RCLCPP_INFO(hello_moveit_logger,"target orientation = %f,%f,%f,%f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z,target_pose.orientation.w);



  // 随后调用moveGroupInterface对象中的setPoseTarget方法将这个目标姿态设置为机械臂的运动目标
  moveGroupInterface.setPoseTarget(target_pose);
 
  //lambda表达式中生成计划并传入msg，返回到列表中plan
  auto const [success, plan] = [&moveGroupInterface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(moveGroupInterface.plan(msg));
    return std::make_pair(ok, msg);
  }();
 

  auto pose = moveGroupInterface.getCurrentPose();
  RCLCPP_INFO(hello_moveit_logger,"current position    = %f,%f,%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  RCLCPP_INFO(hello_moveit_logger,"current orientation = %f,%f,%f,%f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z,target_pose.orientation.w);

  // 判断标志位，执行plan
  if(success) {
    moveGroupInterface.execute(plan);
  } else {
    RCLCPP_ERROR(hello_moveit_logger, "Planing failed!");
  }
  // 停止异步执行器
  executor.cancel(); // 取消所有待处理的回调
  executorThread.join();  // 等待异步线程结束

  // 关闭ROS 
  rclcpp::shutdown();
  return 0;
}