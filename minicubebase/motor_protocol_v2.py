from dynamixel_sdk import *

class MotorV2:
    def __init__(self, DEVICE_NAME, ID):
        self.ID = ID
        self.PROTOCOL_VERSION = 2.0
        self.BAUDRATE = 57600
        self.DIRECTION = 'CW'

        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.ADDR = {
            "TORQUE_ENABLE": 64,
            "GOAL_VELOCITY": 104,
            "GOAL_POSITION": 116,
            "PRESENT_POSITION": 132,
            "OPERATING_MODE": 11,
        }

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.MODES = {
            "VELOCITY_MODE": 1,
            "POSITION_MODE": 3,
            "EXTENDED_POSITION_MODE": 4,
            "PWM_MODE": 16
        }

        # Open port
        if not self.portHandler.openPort():
            print("❌ Failed to open the port!")
        else:
            print("✅ Port opened successfully.")

        # Set baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("❌ Failed to set the baudrate!")
        else:
            print("✅ Baudrate set successfully.")

    def enable_torque(self):
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, self.ADDR["TORQUE_ENABLE"], self.TORQUE_ENABLE)
        print(f"enable_torque -> result: {res}, error: {err}")

    def disable_torque(self):
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, self.ADDR["TORQUE_ENABLE"], self.TORQUE_DISABLE)
        print(f"disable_torque -> result: {res}, error: {err}")

    def set_mode(self, mode):
        self.disable_torque()
        mode_val = self.MODES.get(mode)
        if mode_val is None:
            print(f"❌ Unknown mode: {mode}")
            return
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID, self.ADDR["OPERATING_MODE"], mode_val)
        print(f"set_mode({mode}) -> result: {res}, error: {err}")
        self.enable_torque()

    def move_forward(self, velocity=100):
        self.DIRECTION = "CW"
        res, err = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, self.ADDR["GOAL_VELOCITY"], velocity)
        print(f"move_forward -> result: {res}, error: {err}")

    def move_backward(self, velocity=100):
        self.DIRECTION = "CCW"
        vel_val = 0xFFFFFFFF + 1 + (-velocity)
        res, err = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, self.ADDR["GOAL_VELOCITY"], vel_val)
        print(f"move_backward -> result: {res}, error: {err}")

    def stop_move(self):
        res, err = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, self.ADDR["GOAL_VELOCITY"], 0)
        print(f"stop_move -> result: {res}, error: {err}")

    def move_deg(self, deg):
        initial_pos, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, self.ID,
                                                                 self.ADDR["PRESENT_POSITION"])
        goal_pos = initial_pos + deg
        res, err = self.packetHandler.write4ByteTxRx(self.portHandler, self.ID, self.ADDR["GOAL_POSITION"], goal_pos)
        print(f"move_deg -> result: {res}, error: {err}")
