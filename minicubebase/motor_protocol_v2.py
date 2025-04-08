class MotorV2:
    def __init__(self):
        self.PROTOCOL_VERSION = 2.0
        self.BAUDRATE = 57600

        self.ADDR = {
            "TORQUE_ENABLE": 64,
            "GOAL_POSITION": 116,
            "PRESENT_POSITION": 132,
            "OPERATION_MODE": 1,
            "GOAL_VELOCITY": 1
        }

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.MODES = {

        }



