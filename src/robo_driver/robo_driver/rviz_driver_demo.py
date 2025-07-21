#!/usr/bin/env python
# coding:utf-8
"""
rviz机械臂驱动节点
Rviz robot arm driver node
"""
import rclpy
from rclpy.node import Node
from uservo.uservo_ex import uservo_ex
import struct
from sensor_msgs.msg import JointState

# 设置角度话题，用于区分不同节点 / Topic for setting angles, distinguishes nodes
ROBO_SET_ANGLE_SUBSCRIBER = "set_angle_topic" + str(uservo_ex.ID)


class Arm_contorl(Node):

    def __init__(self):
        # 初始化节点 / Initialize node
        super().__init__("robo_driver_node4rviz")  # 节点名称 / Node name
        # 声明参数并获取机器人类型 / Declare parameter and get robot type
        self.declare_parameter("robo_type", "robo_770")
        self.robo_type = (
            self.get_parameter("robo_type").get_parameter_value().string_value
        )
        # 初始化舵机控制器 / Initialize servo controller
        try:
            self.Servo = uservo_ex(self.robo_type)
        except ValueError as e:
            print(e)
            raise
        # 目标角度列表 / Initial target angles
        self.target_angle = self.Servo.ZERO_ANGLE

        # 设置默认速度 / Default speed for each servo
        self.speed = [50 for _ in range(self.Servo.srv_num)]
        # 当前舵机角度 / Current servo angles
        self.current_angle = [0.0 for _ in range(self.Servo.srv_num)]
        # 根据机器人类型订阅不同话题 / Subscribe based on robot type
        if self.robo_type == "robo_770":
            # 订阅joint_states话题 / Subscribe to joint_states topic
            self.angle_subscribers = self.create_subscription(
                JointState,
                "joint_states",
                self.set_angle_callback_robo_770,
                10,
            )
        # 初始化完成日志 / Initialization complete log
        self.get_logger().info("初始化完成 / Initialization complete")

    def node_close(self):
        # 关闭节点时，停止所有舵机 / Stop all servos when node closes
        self.Servo.servo_all_stop()

    # 新的执行命令 / Callback for new angle commands
    def set_angle_callback_robo_770(self, msg):
        for i in range(len(msg.name)):
            # 跳过第7个关节 / Skip joint7_left
            if msg.name[i] == "joint7_left":
                continue
            # 获取舵机ID / Get servo ID
            id = self.Servo.INDEX_JOINT_[msg.name[i]]
            # 计算目标角度 / Compute target servo angle
            self.target_angle[id] = self.Servo.robo770_jointstate2servoangle(
                id, msg.position[i]
            )
        # 按速度控制执行 / Execute by velocity control
        self.arm_move_by_velocity()

    # 控制by速度 / Control by velocity
    def arm_move_by_velocity(self):
        # 构造角度列表 / Prepare angles
        set_angle = [0.0 for _ in range(self.Servo.srv_num)]
        for i in range(self.Servo.srv_num):
            set_angle[i] = self.target_angle[i]
        # 打包命令数据 / Pack command data
        command_data_list = [
            struct.pack(
                "<BhHHHH", i, int(set_angle[i] * 10), 250, 20, 20, 0
            )
            for i in range(self.Servo.srv_num)
        ]
        # 发送速度控制命令 / Send velocity control commands
        self.Servo.set_angle_by_velocity(self.Servo.srv_num, command_data_list)


def main(args=None):
    # 初始化ROS客户端 / Initialize ROS client
    rclpy.init(args=args)
    try:
        robo_driver_node = Arm_contorl()
    except Exception as e:
        return

    try:
        # 进入循环 / Spin node
        rclpy.spin(robo_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭并销毁节点 / Close and destroy node
        robo_driver_node.node_close()
        robo_driver_node.destroy_node()


if __name__ == "__main__":
    main()
