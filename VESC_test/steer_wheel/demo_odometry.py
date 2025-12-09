import time
import math
import sys
import os

# 确保能找到项目根目录以进行绝对导入
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(current_dir))
if project_root not in sys.path:
    sys.path.append(project_root)

# 确保能找到同级目录的模块
if current_dir not in sys.path:
    sys.path.append(current_dir)

from VESC_test.steer_wheel.Steering_wheel_chassis import VESCMonitor, BasicConfig
from VESC_test.steer_wheel.chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics, Odometry, WheelState

def main():
    # 1. 配置参数
    WHEEL_RADIUS = 0.075  # 75mm
    GEAR_RATIO = 1.0      # 轮毂电机通常直驱，减速比1:1，如果是减速电机请修改
    
    # 2. 初始化 VESC 监控
    print("正在初始化 VESC CAN 监控...")
    monitor = VESCMonitor()
    monitor.start()
    
    # 3. 初始化运动学模型
    geo = ChassisGeometry(length=0.6, width=0.5, wheel_radius=WHEEL_RADIUS)
    kinematics = FourWheelSteeringKinematics(geo)
    odom = Odometry()
    
    # ID 到位置名称的映射
    id_map = {
        BasicConfig.FL_STEER_ID: ("FL", "steer"),
        BasicConfig.FR_STEER_ID: ("FR", "steer"),
        # BasicConfig.RL_STEER_ID: ("RL", "steer"),
        # BasicConfig.RR_STEER_ID: ("RR", "steer"),
        BasicConfig.FL_DRIVE_ID: ("FL", "drive"),
        BasicConfig.FR_DRIVE_ID: ("FR", "drive"),
        # BasicConfig.RL_DRIVE_ID: ("RL", "drive"),
        # BasicConfig.RR_DRIVE_ID: ("RR", "drive"),
    }
    
    # 存储当前的物理状态
    # key: "FL", "FR", "RL", "RR"
    current_wheel_data = {
        "FL": {"speed": 0.0, "angle": 0.0},
        "FR": {"speed": 0.0, "angle": 0.0},
        "RL": {"speed": 0.0, "angle": 0.0},
        "RR": {"speed": 0.0, "angle": 0.0},
    }
    
    print("开始里程计循环 (按 Ctrl+C 停止)...")
    try:
        while True:
            cycle_start = time.time()
            
            # --- 1. 获取数据并转换为物理单位 ---
            for motor_id in BasicConfig.get_all_ids():
                state = monitor.get_state(motor_id)
                if not state:
                    continue
                
                if motor_id not in id_map:
                    continue
                    
                pos_name, role = id_map[motor_id]
                
                if role == "drive":
                    # RPM -> m/s
                    # v = (RPM / 60) * 2 * pi * R
                    rpm = state.get("rpm", 0.0)
                    speed_mps = (rpm / 60.0) * 2 * math.pi * WHEEL_RADIUS / GEAR_RATIO
                    current_wheel_data[pos_name]["speed"] = speed_mps
                    
                elif role == "steer":
                    # Degrees -> Radians
                    # 注意：这里使用的是 BasicConfig 中处理过的 total_angle (如果有) 
                    # 或者是原始 pid_pos。Steering_wheel_chassis.py 中 monitor 计算了 total_angle
                    # 但对于转向角度，我们需要的是相对于车体正前方的角度 (-180 ~ 180)
                    # monitor.motor_states 里的 'total_angle' 是累积角度。
                    # 我们这里直接用 pid_pos (0-360) 减去偏置，并归一化到 -pi ~ pi
                    
                    raw_pos = state.get("pid_pos", 0.0)
                    offset = BasicConfig.get_offset(motor_id)
                    
                    # 假设 raw_pos 是度数。
                    # 减去偏置
                    angle_deg = raw_pos - offset
                    
                    # 归一化到 -180 ~ 180
                    while angle_deg > 180: angle_deg -= 360
                    while angle_deg < -180: angle_deg += 360
                    
                    angle_rad = math.radians(angle_deg)
                    current_wheel_data[pos_name]["angle"] = angle_rad

            # --- 2. 构建 WheelState 对象 ---
            wheel_states = {}
            for name, data in current_wheel_data.items():
                wheel_states[name] = WheelState(
                    speed_mps=data["speed"],
                    angle_rad=data["angle"]
                )
            
            # --- 3. 运动学解算 ---
            vx, vy, omega = kinematics.forward_kinematics(wheel_states)
            
            # --- 4. 里程计更新 ---
            odom.update(vx, vy, omega, cycle_start)
            
            # --- 5. 显示结果 ---
            x, y, theta = odom.get_pose()
            
            # 简单的清屏并打印 (或直接滚动打印)
            # print(f"\033[H\033[J") # 清屏 (在某些终端有效)
            print(f"Pose -> X: {x:.3f} m, Y: {y:.3f} m, Theta: {math.degrees(theta):.1f} deg | "
                  f"Vel -> Vx: {vx:.2f}, Vy: {vy:.2f}, Omega: {omega:.2f}")
            
            # 控制循环频率
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n停止监控...")
    finally:
        monitor.stop()

if __name__ == "__main__":
    main()
