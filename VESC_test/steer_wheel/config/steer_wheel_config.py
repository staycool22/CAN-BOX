import math
import json
import os
import sys

class Config:
    def __init__(self):
        self.load_defaults()
        self.load_from_files()

    def load_defaults(self):
        # Default values in case loading fails or for optional parameters
        # Set all attributes to None or reasonable defaults
        self.FL_STEER_ID = 39
        self.FR_STEER_ID = 38
        self.RL_STEER_ID = 105
        self.RR_STEER_ID = 106
        self.FL_DRIVE_ID = 32
        self.FR_DRIVE_ID = 34
        self.RL_DRIVE_ID = 107
        self.RR_DRIVE_ID = 108
        self.STEER_ZERO_PARAMS = {}
        self.USE_CURRENT_AS_ZERO = False
        self.ENABLE_DRIVE = True
        self.CAN_CHANNEL_ZERO = 0
        self.CAN_CHANNEL_ONE = 1
        self.CAN_ZERO_BAUD_RATE = 1000000
        self.CAN_ZERO_USE_CANFD = True
        self.CAN_ZERO_DATA_BITRATE = 4000000
        self.CAN_ONE_BAUD_RATE = 1000000
        self.CAN_ONE_USE_CANFD = True
        self.CAN_ONE_DATA_BITRATE = 4000000
        self.SAMPLE_POINT = 75.0
        self.DATA_SAMPLE_POINT = 80.0
        self.STEER_REDUCTION_RATIO = 20.0
        self.STEER_INVERTED_IDS = []
        self.STEER_ANGLE_LIMITS = None
        self.STEER_KP = 35.0
        self.STEER_ANGLE_TOLERANCE = 1.0
        self.DRIVE_WHEEL_RADIUS = 0.085
        self.DRIVE_REDUCTION_RATIO = 1.0
        self.DRIVE_POLE_PAIRS = 15
        self.MAX_RPM_REF = 1500.0
        self.CHASSIS_LENGTH = 0.20
        self.CHASSIS_WIDTH = 0.30
        self.ENABLE_ACKERMANN_MODE = False
        self.ACKERMANN_4WS = True
        self.ENABLE_WHEEL_GROUP_CAN_MODE = False
        self.WHEEL_GROUP_CAN_MAPPING = {}

    def load_from_files(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Helper to load json
        def load_json(filename):
            path = os.path.join(base_dir, filename)
            if os.path.exists(path):
                with open(path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            return {}

        # Load Hardware Config
        hw_data = load_json('config_hardware.json')
        for k, v in hw_data.items():
            if k == "STEER_ZERO_PARAMS":
                # Convert keys from string to int
                v = {int(mk): mv for mk, mv in v.items()}
            elif k == "WHEEL_GROUP_CAN_MAPPING":
                v = {int(mk): mv for mk, mv in v.items()}
            setattr(self, k, v)

        # Load Control Config
        ctrl_data = load_json('config_control.json')
        for k, v in ctrl_data.items():
            setattr(self, k, v)

    # Helper methods
    def get_steer_ids(self):
        return [self.FL_STEER_ID, self.FR_STEER_ID, self.RL_STEER_ID, self.RR_STEER_ID]
    
    def get_drive_ids(self):
        return [self.FL_DRIVE_ID, self.FR_DRIVE_ID, self.RL_DRIVE_ID, self.RR_DRIVE_ID]

    def get_all_ids(self):
        return self.get_steer_ids() + self.get_drive_ids()

# Create singleton instance
config = Config()
BasicConfig = config # Alias for backward compatibility if needed temporarily, but I will replace usages.
