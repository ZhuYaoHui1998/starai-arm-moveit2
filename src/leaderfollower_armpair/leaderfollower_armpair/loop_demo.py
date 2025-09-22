#!/usr/bin/env python
#coding:utf-8
'''
从动臂舵机控制节点(Demo)
'''
import rclpy
from rclpy.node import Node
from uservo.uservo_ex import uservo_ex
import struct
import random
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

CHECK_BLOCK_CATCH = 1
SHOW_MODE = 0
LEADER_ARM_ANGLE_TOPIC = 'leader_arm_angle_topic' + str(uservo_ex.ID)
FOLLOWER_ARM_ANGLE_TOPIC = 'follower_arm_angle_topic' + str(uservo_ex.ID)
ANGLE_DEAD = -72.0


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp              # 比例系数
        self.ki = ki              # 积分系数
        self.kd = kd              # 微分系数
        self.prev_error = 0       # 上一次误差
        self.integral = 0         # 积分值

    def compute(self, setpoint,measured_value, dt):
        # 计算误差
        error = setpoint - measured_value
        # 计算积分
        self.integral += error * dt
        # 计算微分
        derivative = (error - self.prev_error) / dt
        # 计算 PID 输出
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        if output >1:
            output = 1
        elif output <-1:
            output = -1
        # 更新上一次误差
        self.prev_error = error
        return setpoint + output

    def reset(self):
        self.prev_error = 0
        self.integral = 0


class FollowerArm(Node):

    def __init__(self):
        self.minangle = 0.0
        random_suffix = str(random.randint(0, 10000))
        super().__init__(f'follower_arm_node_{random_suffix}')
        self.declare_parameter('PORT_NAME', '/dev/ttyUSB0')
        self.PORT_NAME = self.get_parameter('PORT_NAME').get_parameter_value().string_value
        self.enable = True

        try:
            self.Servo = uservo_ex('robo_770',PORT_NAME = self.PORT_NAME,SET_ZERO = False)
        except ValueError as e:
            raise
        self.subscription = self.create_subscription(
            Float32MultiArray,
            LEADER_ARM_ANGLE_TOPIC,
            self.set_servo_angle_callback,
            1)
        self.angle_publishers = self.create_publisher(
            Float32MultiArray,                                               
            FOLLOWER_ARM_ANGLE_TOPIC,
            1)
        self.joint_states_publisher = self.create_publisher(JointState, "joint_states", 10)

        self.subscription
        self.pid = PID(0.2,0.0,0.0)



        self.current_angle = [0.0 for _ in range(self.Servo.srv_num)]
        self.servo_id_list = [ i for i in range(self.Servo.srv_num)]
        self.interval= [100 for _ in range(self.Servo.srv_num)]
        self.target_angle = [0.0 for _ in range(self.Servo.srv_num)]

        # timer_period = 0.050  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_servo_angle_callback(self,msg):
        if self.enable:
            for id in range(self.Servo.srv_num):
                self.target_angle[id] = msg.data[id]
            self.publish_current_angle()
            self.arm_move_by_velocity()

    # def timer_callback(self):
    #     if self.enable:
    #         self.publish_current_angle()


    def publish_current_angle(self):

        msg = Float32MultiArray()
        msg.data = [0.0 for _ in range(self.Servo.srv_num)]
        # print("reading")
        self.Servo.uservo.send_sync_servo_monitor(self.Servo.servo_ids)
        # for i in range(self.Servo.srv_num):
        #     self.current_angle[i] = self.Servo.uservo.servos[i].angle_monitor
        #     msg.data[i] = self.Servo.uservo.servos[i].angle_monitor
        # self.angle_publishers.publish(msg)


        if CHECK_BLOCK_CATCH :
            self.check_gripper(self.Servo.uservo.servos[6].angle_monitor,self.Servo.uservo.servos[6].status)

        if SHOW_MODE:
            JointState_msg = JointState()
            JointState_msg.header.stamp = self.get_clock().now().to_msg()
            JointState_msg.velocity = []
            JointState_msg.effort = []
            for i in range(7):
                self.current_angle[i] = self.Servo.uservo.servos[i].angle_monitor
                JointState_msg.name.append(self.Servo.JOINT_[i])
                JointState_msg.position.append(self.Servo.robo770_servoangle2jointstate(servo_id=i, servo_angle=self.current_angle[i]))

            self.joint_states_publisher.publish(JointState_msg)


    def arm_move_by_velocity(self):

        if not self.compare_arrays():
            return

        set_angle = [0.0 for _ in range(self.Servo.srv_num)]
        for i in range(self.Servo.srv_num):
            # set_angle[i] = self.pid.compute(self.target_angle[i],self.current_angle[i],0.01)
            set_angle[i] = self.target_angle[i]
        command_data_list = [struct.pack('<BhHH',self.servo_id_list[i],int(set_angle[i]*10), 100, 0)for i in range(self.Servo.srv_num-1)]
        self.Servo.set_angle(self.Servo.srv_num-1,command_data_list)

        if set_angle[6] <= -60 :
            self.Servo.gripper_mode = 1
        elif set_angle[6] >= -10 :
            self.Servo.gripper_mode = 0

        if self.Servo.gripper_mode != self.Servo.last_gripper_mode:
            if self.Servo.gripper_mode:
                command_data_list = [struct.pack('<BhHH',6,int((ANGLE_DEAD-5)*10), 100, 0)]
                self.Servo.set_angle(1,command_data_list)
            else :
                command_data_list = [struct.pack('<BhHH',6,int(0), 100, 0)]
                self.Servo.set_angle(1,command_data_list)
        self.Servo.last_gripper_mode = self.Servo.gripper_mode



    def node_close(self):
        # self.Servo.servo_all_stop()
        pass

    def check_gripper(self,angle,status):
        if angle < self.minangle:
            self.minangle = angle
        print(self.minangle)
        if angle < ANGLE_DEAD:
            if (status & 0b100) == 0:
                self.enable = False
                self.Servo.move_to_zero()
                self.Servo.move_to_sleep()
                self.Servo.servo_all_stop()
                self.node_close()
                self.destroy_node()
                raise ValueError(f"未捕捉到方块")

    def compare_arrays(self):
        """
        检查两个7元素数组对应元素的绝对差是否全部<=5.0
        :param A: 第一个浮点数数组（长度7）
        :param B: 第二个浮点数数组（长度7） 
        :return: bool 所有元素差值<=5.0返回True，否则False
        """
        for i in range(6):
            if abs(self.Servo.uservo.servos[i].angle_monitor - self.target_angle[i]) >= 30.0:
                print("error ",self.Servo.uservo.servos[i].angle_monitor,self.target_angle[i])
                return False
        return True



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
        follower_arm_node.Servo.servo_all_stop()
        follower_arm_node.node_close()
        follower_arm_node.destroy_node()




if __name__ == '__main__':
    main()
