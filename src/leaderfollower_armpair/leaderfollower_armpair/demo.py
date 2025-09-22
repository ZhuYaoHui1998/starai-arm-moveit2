#!/usr/bin/env python
#coding:utf-8
'''
主从直接控制节点
'''
import rclpy
from rclpy.node import Node
from uservo.uservo_ex import uservo_ex
import struct
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import Float32MultiArray


LEADER_ARM_ANGLE_TOPIC = 'leader_arm_angle_topic' + str(uservo_ex.ID)
# FOLLOWER_ARM_ANGLE_TOPIC = 'follower_arm_angle_topic' + str(uservo_ex.ID)



class FollowerArm(Node):
    last_button = False
    def __init__(self):
        super().__init__('arm_node')
        self.declare_parameter('robo_type', 'robo_770')
        self.declare_parameter('leader', '/dev/ttyUSB0')
        self.declare_parameter('follower', '/dev/ttyUSB1')
        self.leader_port = self.get_parameter('leader').get_parameter_value().string_value
        self.follower_port = self.get_parameter('follower').get_parameter_value().string_value

        try:
            self.leader_Servo = uservo_ex('robo_770',PORT_NAME = self.leader_port,SET_ZERO = False)
            self.follower_Servo = uservo_ex('robo_770',PORT_NAME = self.follower_port,SET_ZERO = False)

        except ValueError as e:
            raise
                # 创建话题：发布joint_states
        self.joint_states_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.angle_publishers = self.create_publisher(
            Float32MultiArray,                                               
            LEADER_ARM_ANGLE_TOPIC,
            1)
        # self.subscription
        timer_period = 0.040  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.current_angle = [0.0 for _ in range(self.leader_Servo.srv_num)]
        self.servo_id_list = [ i for i in range(self.leader_Servo.srv_num)]
        self.interval= [100 for _ in range(self.leader_Servo.srv_num)]
        self.target_angle = [0.0 for _ in range(self.leader_Servo.srv_num)]

    def timer_callback(self):
        self.query_servo_current_angle()
        self.publish_current_angle()
        self.arm_move_by_velocity()

    def query_servo_current_angle(self):
        self.leader_Servo.uservo.send_sync_servo_monitor(self.leader_Servo.servo_ids)

        msg = Float32MultiArray()
        msg.data = [0.0 for _ in range(7)]

        for i in range(7):
            msg.data[i] = self.leader_Servo.uservo.servos[i].angle_monitor
        self.angle_publishers.publish(msg)

        if self.leader_Servo.uservo.button_is_on != self.last_button:
            if self.leader_Servo.uservo.button_is_on:
                self.leader_Servo.servo_stop_lock(0)
                self.leader_Servo.servo_stop_lock(1)
                self.leader_Servo.servo_stop_lock(2)
                self.leader_Servo.servo_stop_lock(3)
                self.leader_Servo.servo_stop_lock(4)
                self.leader_Servo.servo_stop_lock(5)
                # self.leader_Servo.servo_stop_lock(6)

            else:
                self.leader_Servo.servo_keep_lock(0)
                self.leader_Servo.servo_keep_lock(1)
                self.leader_Servo.servo_keep_lock(2)
                self.leader_Servo.servo_keep_lock(3)
                self.leader_Servo.servo_keep_lock(4)
                self.leader_Servo.servo_keep_lock(5)
                # self.leader_Servo.servo_keep_lock(6)
        
        for i in range(7):
            self.target_angle[i] = self.leader_Servo.uservo.servos[i].angle_monitor
        self.last_button = self.leader_Servo.uservo.button_is_on

    def arm_move_by_velocity(self):
        set_angle = [0.0 for _ in range(7)]

        for i in range(7):
            set_angle[i] = self.target_angle[i]
        command_data_list = [struct.pack('<BhHH',self.servo_id_list[i],int(set_angle[i]*10), 100, 0)for i in range(7)]
        self.follower_Servo.set_angle(7,command_data_list)

        # command_data_list = [struct.pack("<BhHHHH", i, int(set_angle[i]*10), 2000, 20, 20, 0)for i in range(7)]

        # self.follower_Servo.set_angle_by_velocity(7,command_data_list)

    def node_close(self):
        self.follower_Servo.servo_all_stop()
        self.leader_Servo.servo_all_stop()

    def publish_current_angle(self):
        JointState_msg = JointState()
        JointState_msg.header.stamp = self.get_clock().now().to_msg()
        JointState_msg.velocity = []
        JointState_msg.effort = []

        # ret = self.follower_Servo.uservo.button_ping()

        print(self.leader_Servo.uservo.button_is_on)

        self.follower_Servo.uservo.send_sync_servo_monitor(self.follower_Servo.servo_ids)

        for i in range(7):
            self.current_angle[i] = self.follower_Servo.uservo.servos[i].angle_monitor
            JointState_msg.name.append(self.follower_Servo.JOINT_[i])
            JointState_msg.position.append(self.follower_Servo.robo770_servoangle2jointstate(servo_id=i, servo_angle=self.current_angle[i]))

        self.joint_states_publisher.publish(JointState_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        follower_arm_node = FollowerArm()
    except Exception as e:
        print(e)
        return
    try:
        rclpy.spin(follower_arm_node)
    except Exception as e:
        return
    finally:
        # follower_arm_node.node_close()
        follower_arm_node.destroy_node()




if __name__ == '__main__':
    main()
