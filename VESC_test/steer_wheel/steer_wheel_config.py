import math

class BasicConfig:
    # VESC ID 配置
    FL_STEER_ID = 39  # 左前转向电机
    FR_STEER_ID = 38  # 右前转向电机
    RL_STEER_ID = 105  # 左后转向电机
    RR_STEER_ID = 106  # 右后转向电机

    FL_DRIVE_ID = 32  # 左前轮毂电机
    FR_DRIVE_ID = 34  # 右前轮毂电机
    RL_DRIVE_ID = 107  # 左后轮毂电机
    RR_DRIVE_ID = 108  # 右后轮毂电机

    # --- 新增：绝对零位校准参数 ---
    # 定义轮子回正（0度）时，对应的【电机圈数】和【编码器角度(0-360)】
    # 格式: { MOTOR_ID: (ZERO_TURNS, ZERO_ENC_ANGLE) }
    STEER_ZERO_PARAMS = {
        39: (0, 75.74), 
        38: (0, 19.20) 
    }
    
    # 是否使用上电时的当前位置作为零点
    # True: 上电时将当前位置记为 0 度 (忽略 STEER_ZERO_PARAMS)
    # False: 使用 STEER_ZERO_PARAMS 作为绝对零点
    USE_CURRENT_AS_ZERO = False

    # 转向电机（用于角度跟踪）
    @classmethod
    def get_steer_ids(cls):
        # return [cls.FL_STEER_ID, cls.FR_STEER_ID, cls.RL_STEER_ID, cls.RR_STEER_ID]
        # 暂时只启用前两个转向电机
        return [cls.FL_STEER_ID, cls.FR_STEER_ID]
    
    @classmethod
    def get_drive_ids(cls):
        # return [cls.FL_DRIVE_ID, cls.FR_DRIVE_ID, cls.RL_DRIVE_ID, cls.RR_DRIVE_ID]
        # 暂时只启用前两个驱动电机
        return [cls.FL_DRIVE_ID, cls.FR_DRIVE_ID]

    @classmethod
    def get_all_ids(cls):
        return cls.get_steer_ids() + cls.get_drive_ids()

    # CAN 配置
    ENABLE_DRIVE = True # 屏蔽驱动电机控制
    DRIVE_CAN_CHANNEL = 0 # 驱动电机 (CAN 2.0, 500k)
    STEER_CAN_CHANNEL = 1 # 转向电机 (CAN FD, 1M/4M)
    
    # 驱动电机 CAN 参数 (CAN FD)
    DRIVE_BAUD_RATE = 1000000
    DRIVE_USE_CANFD = True
    DRIVE_DATA_BITRATE = 4000000
    
    # 转向电机 CAN 参数 (CAN FD)
    STEER_BAUD_RATE = 1000000
    STEER_USE_CANFD = True
    STEER_DATA_BITRATE = 4000000
    
    # CAN FD 特定参数 (保留旧兼容性，如果有其他地方用到)
    SAMPLE_POINT = 75.0
    DATA_SAMPLE_POINT = 80.0
    
    # 转向电机减速比 (电机转 8 圈 = 轮子转 1 圈)
    STEER_REDUCTION_RATIO = 20.0

    STEER_INVERTED_IDS = [38]

    # 转向电机限位保护
    # 当目标角度在死区范围内时，自动翻转驱动方向
    # 格式: (MIN_ANGLE, MAX_ANGLE) 
    # 例如 (90, 270) 表示如果不方便转到 180 度，就转到 0 度并反转驱动
    STEER_ANGLE_LIMITS = None # 暂时禁用，全向转向

    # 转向位置环 PID 参数 (简单 P 控制)
    STEER_KP = 35.0 # 误差 1 度 (Motor) -> 50 RPM (Increased from 27.0)
    # 轮子误差 1 度 -> 电机误差 20 度 -> 输出 1000 RPM
    
    # 转向角度容差 (度)
    # 当轮子角度误差小于此值时，不再进行 PID 调整，而是锁定位置
    STEER_ANGLE_TOLERANCE = 0.5


    # 驱动轮参数
    DRIVE_WHEEL_RADIUS = 0.85 # 米
    DRIVE_REDUCTION_RATIO = 1.0 # 假设为 1:1，如有减速箱请修改
    DRIVE_POLE_PAIRS = 15 # 极对数
    
    # 驱动电机最大参考转速 (用于计算加减速时间)
    # 假设 1000 RPM 对应满速控制量
    MAX_RPM_REF = 1500.0
