import rclpy                            
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
import time
from std_msgs.msg import Float32MultiArray
# from uservo.uservo_ex import uservo_ex
from robo_interfaces.msg import SetAngle
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
ROBO_ACTION_NODE = 'viola_controller_node'
ROBO_CURRENT_ANGLE_SUBSCRIPTION = "joint_states"
ROBO_SET_ANGLE_PUBLISHER ='set_angle_topic'
ROBO_ARM_ACTION_SERVER = '/arm_controller/follow_joint_trajectory'
ROBO_GIRRPER_ACTION_SERVER = '/hand_controller/gripper_cmd'
from sensor_msgs.msg import JointState

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

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # 比例系数
        self.Ki = Ki  # 积分系数
        self.Kd = Kd  # 微分系数
        self.setpoint = setpoint  # 设定值
        self.previous_error = 0  # 上一个误差
        self.integral = 0  # 积分值

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value  # 计算当前误差
        self.integral += error * dt  # 更新积分值
        derivative = (error - self.previous_error) / dt  # 计算微分项
        # PID控制公式
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error  # 更新上一个误差
        return output



def radians_to_degrees( radians):
    degrees = radians * (180 / math.pi)
    return degrees
def meters_to_degrees(meters):
    degrees = (meters / 0.032) * 100
    return degrees
def jointstate2servoangle(servo_id, joint_state):
    if servo_id == 0:
        return radians_to_degrees(joint_state)
    elif servo_id == 1:
        return radians_to_degrees(joint_state)
    elif servo_id == 2:
        return radians_to_degrees(joint_state)
    elif servo_id == 3:
        return radians_to_degrees(joint_state)
    elif servo_id == 4:
        return radians_to_degrees(joint_state)
    elif servo_id == 5:
        return radians_to_degrees(joint_state)
    elif servo_id == 6:
        return meters_to_degrees(joint_state)+100

class RoboActionClient(Node):
    current_angle = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    current_joint_state = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    last_time = 0



    def __init__(self):
        super().__init__(ROBO_ACTION_NODE)

        self.callback_group = ReentrantCallbackGroup()
        # 创建手臂动作服务器
        self.Arm_FollowJointTrajectoryNode = ActionServer(
            self,
            FollowJointTrajectory,
            ROBO_ARM_ACTION_SERVER,
            execute_callback = self.arm_execute_callback,
            cancel_callback = self.arm_cancel_callback,
            callback_group=self.callback_group
        )
        #创建夹爪动作服务器
        self.Hand_GripperCommandNode = ActionServer(
            self,
            GripperCommand,
            ROBO_GIRRPER_ACTION_SERVER,
            execute_callback = self.gripper_execute_callback,
            cancel_callback = self.gripper_cancel_callback,
            callback_group=self.callback_group
        )
        # 创建话题 :接收current_angle_topic消息
        self.currentangle_subscription = self.create_subscription(
            JointState,                                               
            ROBO_CURRENT_ANGLE_SUBSCRIPTION,
            self.current_angle_callback,
            1,
            callback_group=self.callback_group)

        #发布角度控制话题
        self.set_angle_publishers = self.create_publisher(
            SetAngle,ROBO_SET_ANGLE_PUBLISHER,
            1)

        self.get_logger().info(f'{ROBO_ACTION_NODE} is ready.')

    def arm_cancel_callback(self,cancel_request):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

        
    # 接收机械臂当前角度话题回调
    def current_angle_callback(self, msg):
        _data = msg
        for i in range(len(_data.name)):
            joint_id = ROBO_TYPE_1_INDEX_JOINT_[_data.name[i]]
            self.current_angle[joint_id] = jointstate2servoangle(servo_id = joint_id, joint_state =_data.position[i])






    # 处理动作服务器回调
    def arm_execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self.get_logger().info(f'Receiving trajectory with {len(trajectory.points)} points.')
        self.last_time = 0
        finally_targect = [0 for i in range(6)]

        for index,point in enumerate(trajectory.points):
            position = point.positions
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec/1e9
            run_time = time_from_start - self.last_time
            if run_time < 0.1:
                run_time = 0.1
            start_time = time.time()

            goal_msg = SetAngle()
            goal_msg.servo_id = [0,1,2,3,4,5]
            goal_msg.target_angle = [0.0,0.0,0.0,0.0,0.0,0.0]
            goal_msg.time = [2000,2000,2000,2000,2000,2000]
            for i in range(len(trajectory.joint_names)):
                id = ROBO_TYPE_1_INDEX_JOINT_[trajectory.joint_names[i]]
                goal_msg.target_angle[id] = jointstate2servoangle(servo_id = id, joint_state = position[i])
                goal_msg.time[id] = int(run_time*1000)
            self.last_time = time_from_start
            self.set_angle_publishers.publish(goal_msg)

            time.sleep(run_time)

        # # 使用PID，完成轨迹规划
        # self.arm_pid = [PID(1, 0.001, 0.001),
        #         PID(1.5, 1, 0.001),
        #         PID(1.2, 1, 0.001),
        #         PID(1, 0.001, 0.001),
        #         PID(1.5, 2, 0.001),
        #         PID(1, 0.001, 0.001),
        #         PID(1, 0.001, 0.001)]
        # for id in range(6):
        #     finally_targect[id] = goal_msg.target_angle[id]
        #     self.arm_pid[id].setpoint = finally_targect[id]
        # start_time = time.time()
        # while 1:
        #     if time.time() - start_time > 1:
        #         break
        #     for id in range(6):
        #         goal_msg.target_angle[id] = int(self.current_angle[id] + self.arm_pid[id].update(self.current_angle[id], 0.05)) 
        #         goal_msg.time[id] = int(500)

        #     self.set_angle_publishers.publish(goal_msg)
        #     time.sleep(0.05)

        # 成功完成所有点，返回成功状态
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

    def gripper_cancel_callback(self,cancel_request):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def gripper_execute_callback(self, goal_handle):
        position = goal_handle.request.command.position 
        # max_effort = goal_handle.request.command.max_effort
        # print(f"position:{position}")
        goal_msg = SetAngle()
        goal_msg.servo_id.append(6)
        goal_msg.target_angle.append(jointstate2servoangle(servo_id = 6, joint_state = position))
        goal_msg.time.append(1000)
        # print(f"max_effort:{max_effort}")
        self.set_angle_publishers.publish(goal_msg)
        goal_handle.succeed()

        result = GripperCommand.Result()
        return result



def main(args=None):
    rclpy.init(args=args)
    action_client = RoboActionClient()
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
