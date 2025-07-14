#!/usr/bin/env python
# coding:utf-8
"""
rviz机械臂驱动节点
"""
import rclpy
from rclpy.node import Node
from uservo.uservo_ex import uservo_ex
import struct
from sensor_msgs.msg import JointState

ROBO_SET_ANGLE_SUBSCRIBER = "set_angle_topic" + str(uservo_ex.ID)


class Arm_contorl(Node):

    def __init__(self):
        super().__init__("robo_driver_node4rviz")
        self.declare_parameter("robo_type", "robo_770")
        self.robo_type = (
            self.get_parameter("robo_type").get_parameter_value().string_value
        )
        try:
            self.Servo = uservo_ex(self.robo_type)
        except ValueError as e:
            print(e)
            raise
        self.target_angle = self.Servo.ZERO_ANGLE

        self.speed = [50 for _ in range(self.Servo.srv_num)]
        self.current_angle = [0.0 for _ in range(self.Servo.srv_num)]
        if self.robo_type == "robo_770":
            self.angle_subscribers = self.create_subscription(
                JointState, "joint_states", self.set_angle_callback_robo_770, 10
            )
        self.get_logger().info(f"初始化完成")

    def node_close(self):
        self.Servo.servo_all_stop()

    # 新的执行命令
    def set_angle_callback_robo_770(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == "joint7_left":
                continue
            id = self.Servo.INDEX_JOINT_[msg.name[i]]
            self.target_angle[id] = self.Servo.robo770_jointstate2servoangle(
                id, msg.position[i]
            )

        self.arm_move_by_velocity()

    # 控制by速度
    def arm_move_by_velocity(self):
        set_angle = [0.0 for _ in range(self.Servo.srv_num)]
        for i in range(self.Servo.srv_num):
            set_angle[i] = self.target_angle[i]
        command_data_list = [
            struct.pack("<BhHHHH", i, int(set_angle[i] * 10), 250, 20, 20, 0)
            for i in range(self.Servo.srv_num)
        ]
        self.Servo.set_angle_by_velocity(self.Servo.srv_num, command_data_list)


def main(args=None):
    rclpy.init(args=args)
    try:
        robo_driver_node = Arm_contorl()
    except Exception as e:
        return

    try:
        rclpy.spin(robo_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        robo_driver_node.node_close()
        robo_driver_node.destroy_node()


if __name__ == "__main__":
    main()
