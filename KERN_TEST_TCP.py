from mks_servo42d_57d_tcp import MksServoTCP
from mks_servo42d_57d import Direction, States, MksServo
from pymodbus.client import ModbusTcpClient
import time
import random
MOVE_TIMEOUT = 30  
SETTLE_POLL_INTERVAL = 0.1
DELAY_TIME = 1
SPEED_MIN = 100 
SPEED_MAX = 2000
ACCEL_MIN = 20
ACCEL_MAX = 250
CONVERTER_IP = "192.168.2.201"   # your RS485-to-ETH converter's IP
CONVERTER_PORT = 4196            # confirm the actual configured port
ADDR = 1
PORT = "COM3"   # e.g. "COM5" on Windows
ABS_MAX = 500000
SPEED = 2000
ACCEL = 250


     
def wait_state(motor: MksServoTCP,state: int = States.STATE_STOP, timeout: float = MOVE_TIMEOUT) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        curr_state = motor.query_motor_status()
        # print("state", curr_state, "speed", (motor.read_speed()))
        if curr_state == state:
            return False
        time.sleep(SETTLE_POLL_INTERVAL)
    print("True")
    return True

if __name__ == "__main__":
    motor = MksServoTCP(CONVERTER_IP, port=CONVERTER_PORT, addr=ADDR, debug=False)
    # motor = MksServo(PORT, addr=ADDR, baudrate=38400, debug=True) 
    motor.set_uart_response(respond_all=True, active_report=False)
    motor.go_home()
    print("Homing...")
    while (wait_state(motor, States.STATE_STOP)):
        pass

    motor.run_position_absolute_axis_retry(ABS_MAX,300)
    print("Maxxing...")
    while (wait_state(motor, States.STATE_STOP)):
        pass

    max_axis = motor.read_encoder_fix()
    print("max_axis", max_axis,motor.read_encoder(), "pulses", motor.read_pulses())
    # max_axis = 416082
    while (1):
            speed = random.randint(SPEED_MIN, SPEED_MAX)
            accel = random.randint(ACCEL_MIN, ACCEL_MAX)
            print("speed:", speed, "accel:", accel)
            motor.run_position_absolute_axis_retry(0, speed, accel)
            # motor.run_position_absolute_axis_retry(0, SPEED, ACCEL)
            while (wait_state(motor, States.STATE_STOP)):
                pass
            print("pos_0:",motor.read_encoder_fix())
            speed = random.randint(SPEED_MIN, SPEED_MAX)
            accel = random.randint(ACCEL_MIN, ACCEL_MAX)
            print("speed:", speed, "accel:", accel)
            motor.run_position_absolute_axis_retry(max_axis, speed, accel)
            # motor.run_position_absolute_axis_retry(max_axis, SPEED, ACCEL)
            while (wait_state(motor, States.STATE_STOP)):
                pass
            print("pos_up:", motor.read_encoder_fix())

