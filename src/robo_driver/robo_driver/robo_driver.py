#!/usr/bin/env python
# coding:utf-8
"""
机械臂驱动节点
"""
import rclpy
from rclpy.node import Node
# from uservo.uservo_ex import uservo_ex
from robo_interfaces.srv import ReadData, WriteData
from robo_interfaces.msg import SetAngle
import struct
from sensor_msgs.msg import JointState
import math
import struct
import fashionstar_uart_sdk as uservo
from robo_interfaces.srv import ReadData, WriteData
import time
import serial

ROBO_DRIVER_NODE = "robo_driver_node" 
ROBO_SET_ANGLE_SUBSCRIBER = "set_angle_topic"
SERVO_PORT_NAME = "/dev/ttyUSB0"  # 舵机串口号 <<< 修改为实际串口号
SERVO_BAUDRATE = 1000000  # 舵机的波特率
ID = 0  # 多手臂时区分话题ID
servo_ids = list()

"""
舵机转手臂控制通用类
"""




class writedata:
    def __init__(self, command, servo_id=255, value=0):
        if command == None:
            return
        WriteData_ = WriteData()
        WriteData_.command = command
        WriteData_.servo_id = servo_id
        WriteData_.value = value
        return WriteData_


class uservo_ex:
    ROBO_TYPE_1 = "viola"
    ROBO_TYPE_1_JOINT_ = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7_left",
    ]
    ROBO_TYPE_1_INDEX_JOINT_ = {
        value: index for index, value in enumerate(ROBO_TYPE_1_JOINT_)
    }



    # 参数
    # PORT_NAME: 设置舵机串口号，默认使用/dev/ttyUSB0，当需要在同一个设备上使用多个机械臂时，需要修改该参数
    # SET_ZERO: 是否回到零位，默认回到零位
    # CHECK_ANGLE: 是否检查上电舵机角度，默认检查，确保初始化状态一致性。
    def __init__(self, robo_type, PORT_NAME=None, SET_ZERO=True, CHECK_ANGLE=True):
        self.ROBO_TYPE = robo_type
        if self.ROBO_TYPE == self.ROBO_TYPE_1:
            self.JOINT_ = self.ROBO_TYPE_1_JOINT_
            self.INDEX_JOINT_ = self.ROBO_TYPE_1_INDEX_JOINT_
        # elif self.ROBO_TYPE == self.ROBO_TYPE_2:
        #     self.JOINT_ = self.ROBO_TYPE_2_JOINT_
        #     self.INDEX_JOINT_ = self.ROBO_TYPE_2_INDEX_JOINT_
        else:
            self.JOINT_ = self.ROBO_TYPE_1_JOINT_
            self.INDEX_JOINT_ = self.ROBO_TYPE_1_INDEX_JOINT_
        self.INDEX_JOINT_ = {value: index for index, value in enumerate(self.JOINT_)}


        # 初始化串口
        success = False
        while not success:
            try:
                if PORT_NAME == None:
                    self.uart = serial.Serial(
                        port=SERVO_PORT_NAME,
                        baudrate=SERVO_BAUDRATE,
                        parity=serial.PARITY_NONE,
                        stopbits=1,
                        bytesize=8,
                        timeout=0,
                    )
                else:
                    self.uart = serial.Serial(
                        port=PORT_NAME,
                        baudrate=SERVO_BAUDRATE,
                        parity=serial.PARITY_NONE,
                        stopbits=1,
                        bytesize=8,
                        timeout=0,
                    )
                success = True  # 如果成功初始化，则设置成功标志
            except serial.SerialException as e:
                print(f"串口初始化失败: {e}")
                time.sleep(0.1)  # 暂停 1 秒后重试
        try:
            self.uservo = uservo.UartServoManager(self.uart)
        except Exception as e:
            raise

        self.servo_ids = [0,1,2,3,4,5,6]
        self.srv_num =7
        self.ZERO_ANGLE = [0 for _ in range(self.srv_num)]
        self.ZERO_ANGLE[6] = 500
        self.reset_multi_turn_angle(0xff)
        time.sleep(0.1)

    # # 返回虚拟关节的名称
    # @classmethod
    # def fake_joint_name(cls):
    #     return "right_joint"

    # 将弧度转为角度
    @classmethod
    def radians_to_degrees(cls, radians):
        degrees = radians * (180 / math.pi)
        return degrees

    # 将米转为角度
    @classmethod
    def meters_to_degrees(cls, meters):
        degrees = (meters / 0.032) * 100
        return degrees

    # 将角度转为弧度
    @classmethod
    def degrees_to_radians(cls, degrees):
        radians = degrees * (math.pi / 180)
        return radians

    # 将角度转为米
    @classmethod
    def degrees_to_meters(cls, degrees):
        meters = (degrees / 100) * 0.032
        return meters


    # @classmethod
    # def jointstate2servoangle(cls, servo_id, joint_state):
    #     if servo_id == 0:
    #         return cls.radians_to_degrees(joint_state)
    #     elif servo_id == 1:
    #         return cls.radians_to_degrees(joint_state)
    #     elif servo_id == 2:
    #         return cls.radians_to_degrees(joint_state)
    #     elif servo_id == 3:
    #         return cls.radians_to_degrees(joint_state)
    #     elif servo_id == 4:
    #         return cls.radians_to_degrees(joint_state)
    #     elif servo_id == 5:
    #         return cls.radians_to_degrees(joint_state)
    #     elif servo_id == 6:
    #         return cls.meters_to_degrees(joint_state)+100


    @classmethod
    # 将舵机角度转换为关节位置
    def servoangle2jointstate(cls, servo_id, servo_angle):
        if servo_id == 0:
            return cls.degrees_to_radians(servo_angle)
        elif servo_id == 1:
            return cls.degrees_to_radians(servo_angle)
        elif servo_id == 2:
            return cls.degrees_to_radians(servo_angle)
        elif servo_id == 3:
            return cls.degrees_to_radians(servo_angle)
        elif servo_id == 4:
            return cls.degrees_to_radians(servo_angle)
        elif servo_id == 5:
            return cls.degrees_to_radians(servo_angle)
        elif servo_id == 6:
            return cls.degrees_to_meters(servo_angle-100)


    # 设置角度
    def set_angle(self, size, command_data_list):
            self.uservo.send_sync_angle(size, command_data_list)


    # 设置角度（指定转速）
    def set_angle_by_interval(self, size, command_data_list):
        self.uservo.send_sync_multiturnanglebyinterval(self.uservo.CODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL,size, command_data_list)

    def set_single_angle(self, servo_id, angle, velocity=30):
            self.uservo.set_servo_angle(servo_id, angle, velocity=velocity)

    # 查询角度
    def query_servo_current_angle(self, servo_id):
        if servo_id in self.uservo.servos:
            return self.uservo.query_servo_angle(servo_id)

    # 失能舵机
    def disable_torque(self, servo_id):
        self.servo_stop(servo_id)

    def enable_torque(self, servo_id):
        current_angle = self.uservo.query_servo_angle(servo_id)
        self.set_single_angle(servo_id, current_angle)

    # 查询一次温度
    def get_temperature(self, servo_id):
        if servo_id in self.uservo.servos:
            return self.uservo.query_temperature(servo_id)

    # 查询错误码
    def get_error_code(self, servo_id):
        if servo_id in self.uservo.servos:
            code = self.uservo.query_status(servo_id)
            if code <= 1:
                return 0
            else:
                return code

    # 重设所有舵机多圈圈数
    def reset_all_servo_multi_turn_angle(self):
        for i in self.uservo.servos:
            self.reset_multi_turn_angle(i)

    # 重设指定舵机多圈圈数
    def reset_multi_turn_angle(self, servo_id):
        self.uservo.disable_torque(servo_id)
        time.sleep(0.05)
        self.uservo.reset_multi_turn_angle(servo_id)

    # 返回舵机数量
    def servo_num(self):
        if self.srv_num:
            return self.srv_num
        return 0

    def servo_stop(self, servo_id, mode=2, power=500):
        self.uservo.stop_on_control_mode(servo_id, mode, power)

    def servo_all_stop(self, mode=2, power=200):
        for id in range(self.srv_num):
            self.servo_stop(id, mode, power)
            time.sleep(0.05)


    #舵机停止释放锁力
    def servo_stop_lock(self, servo_id):
        self.uservo.stop_on_control_mode(servo_id,0,500)
    #舵机保持锁力
    def servo_keep_lock(self, servo_id):
        self.uservo.stop_on_control_mode(servo_id,1,50000)

    #舵机设置原点
    def servo_set_origin_point(self, servo_id):
        self.uservo.set_origin_point(servo_id)
    #发送0°角度命令
    def servo_set_zero_angle(self, servo_id, angle, interval, power):
        self.uservo.set_servo_angle(servo_id,angle,interval,power)

    #读取内存表数据
    def servo_read_data(self, servo_id, address):
        self.uservo.read_data(servo_id,address)

    #写入内存表数据
    def servo_write_data(self, servo_id, address, content):
        self.uservo.write_data(servo_id,address,content)






class Arm_contorl(Node):

    def __init__(self):
        super().__init__(ROBO_DRIVER_NODE)
        self.declare_parameter("robo_type", "robo")
        self.robo_type = (
            self.get_parameter("robo_type").get_parameter_value().string_value
        )
        try:
            self.Servo = uservo_ex(self.robo_type)
        except ValueError as e:
            raise
        self.target_angle = self.Servo.ZERO_ANGLE
        # print(self.target_angle)
        self.interval = [1500 for _ in range(self.Servo.srv_num)]
        # self.speed = [50 for _ in range(self.Servo.srv_num)]
        self.current_angle = [0.0 for _ in range(self.Servo.srv_num)]
        self.arm_move_by_time()

        # 创建话题：发布joint_states
        self.joint_states_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        # 创建话题：处理设置角度
        self.angle_subscribers = self.create_subscription(
            SetAngle, ROBO_SET_ANGLE_SUBSCRIBER, self.set_angle_callback, 10
        )
        self.timer2 = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info(f"初始化完成")

    def node_close(self):
        pass
        # self.Servo.servo_all_stop()

    # 新的执行命令
    def set_angle_callback(self, msg):
        for i in range(len(msg.servo_id)):
            id = msg.servo_id[i]
            self.target_angle[id] = int(10*msg.target_angle[i])
            if int(msg.time[i]) < 40:
                (msg.time[i]) = 40
            self.interval[id] = int(msg.time[i])+100

            self.arm_move_by_time()


    # 定时任务
    def timer_callback(self):
        self.publish_current_angle()

    # 查询舵机角度发布
    def publish_current_angle(self):
        JointState_msg = JointState()
        JointState_msg.header.stamp = self.get_clock().now().to_msg()
        JointState_msg.velocity = []
        JointState_msg.effort = []
        self.Servo.uservo.send_sync_servo_monitor(self.Servo.servo_ids)

        for i in range(self.Servo.srv_num):
            self.current_angle[i] = self.Servo.uservo.servos[i].angle_monitor
            JointState_msg.name.append(self.Servo.JOINT_[i])
            JointState_msg.position.append(
                self.Servo.servoangle2jointstate(
                    servo_id=i, servo_angle=self.current_angle[i]
                )
            )
        self.joint_states_publisher.publish(JointState_msg)

    # 默认：控制by时间
    def arm_move_by_time(self):
        acc = [0.0 for _ in range(self.Servo.srv_num)]
        for i in range(self.Servo.srv_num):
            acc[i] = int(self.interval[i]/2)
        command_data_list = [struct.pack("<BlLHHH", i, self.target_angle[i],self.interval[i],acc[i] , acc[i], 0)for i in range(self.Servo.srv_num)]
        self.Servo.set_angle_by_interval(self.Servo.srv_num, command_data_list)



def main(args=None):
    rclpy.init(args=args)
    try:
        robo_driver_node = Arm_contorl()
    except Exception as e:
        print(e)
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
