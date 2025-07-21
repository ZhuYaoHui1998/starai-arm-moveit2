import fashionstar_uart_sdk as uservo
import serial

SERVO_PORT_NAME = "/dev/ttyUSB0"  # 舵机串口号 <<< 修改为实际串口号
SERVO_BAUDRATE = 1000000  # 舵机的波特率
servo_id = 6 #gripper

def main():
    uart = serial.Serial(
        port=SERVO_PORT_NAME,
        baudrate=SERVO_BAUDRATE,
        parity=serial.PARITY_NONE,
        stopbits=1,
        bytesize=8,
        timeout=0,
    )
    control = uservo.UartServoManager(uart)

    control.set_origin_point(servo_id)



if __name__ == "__main__":
    main()