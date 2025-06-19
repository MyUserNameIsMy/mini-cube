import time

from dynamixel_sdk import *


class MotorV1:
    def __init__(self, DEVICE_NAME, ID, BAUDRATE=57600):
        self.ID = ID
        self.PROTOCOL_VERSION = 1.0
        self.BAUDRATE = BAUDRATE
        print(self.PROTOCOL_VERSION, self.BAUDRATE)
        self.DIRECTION = 'CW'
        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.ADDR = {
            "TORQUE_ENABLE": 24,
            "GOAL_POSITION": 30,
            "PRESENT_POSITION": 36,
            "OPERATION_MODE": 1,
            "GOAL_VELOCITY": 1,
            "MOVING_SPEED": 32,
            "CCW": 6,
            "CW": 8,
        }

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.MODES = {
            "WHEEL_MODE": {
                "CW": 0,
                "CCW": 0,
            },
            "JOINT_MODE": {
                "CW": 0,
                "CCW": 4095,
            },
            "MULTI_TURN_MODE": {
                "CW": 4095,
                "CCW": 4095,
            }
        }

        # Open the port
        if not self.portHandler.openPort():
            print("Failed to open the port!")
            # Depending on your application, you might want to exit or handle this error.
        else:
            print("Port opened successfully.")

        # Set the baudrate for the port
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to set the baudrate!")
        else:
            print("Baudrate set successfully.")

    def enable_torque(self):
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, self.ADDR["TORQUE_ENABLE"],
                                                     self.TORQUE_ENABLE)
        print(f"enable_torque -> result: {self.packetHandler.getTxRxResult(res)}, error: {self.packetHandler.getRxPacketError(err)}")

    def disable_torque(self):
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, self.ADDR["TORQUE_ENABLE"],
                                                     self.TORQUE_DISABLE)
        print(f"disable_torque -> result: {res}, error: {err}")

    def set_mode(self, mode):
        self.disable_torque()
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["CCW"], self.MODES[mode]["CCW"])
        print(
            f"set_mode_ccw -> result: {self.packetHandler.getTxRxResult(res)}, error: {self.packetHandler.getRxPacketError(err)}")
        time.sleep(0.45)
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["CW"], self.MODES[mode]["CW"])
        print(
            f"set_mode_cw -> result: {self.packetHandler.getTxRxResult(res)}, error: {self.packetHandler.getRxPacketError(err)}")
        self.enable_torque()

    def set_speed(self, speed):
        self.disable_torque()
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["MOVING_SPEED"], speed)
        print(
            f"set_speed -> result: {self.packetHandler.getTxRxResult(res)}, error: {self.packetHandler.getRxPacketError(err)}")

        self.enable_torque()
    def move_deg(self, deg):
        initial_pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, self.ID,
                                                                 self.ADDR["PRESENT_POSITION"])
        goal_pos = initial_pos + deg
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["GOAL_POSITION"], goal_pos)
        print(f"move_deg -> result: {res}, error: {err}")

    def set_deg(self, deg):
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["GOAL_POSITION"], deg)
        print(f"move_deg -> result: {res}, error: {err}")


    def move_forward(self):
        self.DIRECTION = "CW"
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["MOVING_SPEED"], 1000)
        print(f"move_forward -> result: {res}, error: {err}")

    def move_backward(self):
        self.DIRECTION = "CCW"
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["MOVING_SPEED"], 2000)
        print(f"move_backward -> result: {res}, error: {err}")

    def stop_move(self):
        if self.DIRECTION == "CW":
            value = 1024
        else:
            value = 0
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["MOVING_SPEED"], value)
        print(f"stop_move -> result: {res}, error: {err}")
