from dynamixel_sdk import *
from pprint import pprint
class MotorV1:
    def __init__(self, DEVICE_NAME, ID):
        self.ID = ID
        self.PROTOCOL_VERSION = 1.0
        self.BAUDRATE = 57600

        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.ADDR = {
            "TORQUE_ENABLE": 24,
            "GOAL_POSITION": 30,
            "PRESENT_POSITION": 36,
            "OPERATION_MODE": 1,
            "GOAL_VELOCITY": 1,
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

    def enable_torque(self):
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, self.ADDR["TORQUE_ENABLE"], self.TORQUE_ENABLE)
        pprint("enable_torque", res, err)

    def disable_torque(self):
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, self.ADDR["TORQUE_ENABLE"], self.TORQUE_DISABLE)
        pprint("disable_torque", res, err)

    def set_mode(self, mode):
        self.disable_torque()
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["CW"], self.MODES[mode]["CW"])
        pprint("set_mode_cw", res, err)
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["CCW"], self.MODES[mode]["CCW"])
        pprint("set_mode_ccw", res, err)
        self.enable_torque()

    def move_deg(self, deg):
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.ID, self.ADDR["GOAL_POSITION"], deg)
        pprint("move_deg", res, err)






