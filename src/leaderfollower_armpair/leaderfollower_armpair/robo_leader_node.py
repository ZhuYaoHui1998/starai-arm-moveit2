#!/usr/bin/env python
#coding:utf-8

'''
操控臂舵机控制节点(Demo)
'''
import rclpy
from rclpy.node import Node
from uservo.uservo_ex import uservo_ex
from std_msgs.msg import Float32MultiArray

LEADER_ARM_ANGLE_TOPIC = 'leader_arm_angle_topic' + str(uservo_ex.ID)


class LeaderArm(Node):

    def __init__(self):
        super().__init__('followerarm_leader')
        self.declare_parameter('robo_type', 'robo')
        self.robo_type = self.get_parameter('robo_type').get_parameter_value().string_value

        try:
            self.Servo = uservo_ex(self.robo_type,SET_ZERO = False)
        except ValueError as e:
            raise
        # 创建主臂角度发布者
        self.angle_publishers = self.create_publisher(
            Float32MultiArray,                                               
            LEADER_ARM_ANGLE_TOPIC,
            1)
        
        # TODO 进阻尼
        self.Servo.servo_all_stop(power = 100)
        # 创建定时器
        timer_period = 0.030  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 初始化舵机管理器
    def timer_callback(self):
        self.publish_current_angle()

    def node_close(self):
        self.Servo.servo_all_stop()

    def publish_current_angle(self):
        msg = Float32MultiArray()
        msg.data = [0.0 for _ in range(self.Servo.srv_num)]

        self.Servo.uservo.send_sync_servo_monitor(self.Servo.servo_ids)
        for i in range(self.Servo.srv_num):
            msg.data[i] = self.Servo.uservo.servos[i].angle_monitor
        self.angle_publishers.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        followerarm_leader = LeaderArm()
    except Exception as e:
        print(e)
        return
    
    try:
        rclpy.spin(followerarm_leader)
    except KeyboardInterrupt:
        pass
    finally:
        followerarm_leader.node_close()
        followerarm_leader.destroy_node()




if __name__ == '__main__':
    main()
