import time
import sys
import os
import argparse

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

from actuators.Steering_wheel_chassis import VESCMonitor
from config.steer_wheel_config import config

def _unlock_motors(monitor, motor_ids):
    """发送零电流指令以解锁电机"""
    print("正在解锁电机 (发送 0A 电流)...")
    for mid in motor_ids:
        # 尝试获取接口
        vesc = monitor.get_vesc_interface(mid)
        if vesc:
            # 发送电流 0
            vesc.send_current(mid, 0.0)
            print(f"  -> ID {mid} 解锁指令已发送")


def _wait_enc2_ready(monitor, motor_ids, timeout):
    t0 = time.time()
    ready = set()
    while time.time() - t0 < timeout:
        all_ready = True
        for mid in motor_ids:
            state = monitor.get_state(mid)
            v = state.get("enc2")
            if v is None:
                all_ready = False
                break
            else:
                ready.add(mid)
        if all_ready and len(ready) == len(motor_ids):
            return True
        time.sleep(0.02)
    return False

def _capture_enc2_zero(monitor, motor_id, all_ids=None):
    if all_ids is None:
        all_ids = [motor_id]
        
    while True:
        # 构建状态显示字符串
        status_lines = []
        for mid in all_ids:
            state = monitor.get_state(mid)
            enc1 = state.get("last_pos")
            enc2 = state.get("enc2")
            
            enc1_str = f"{enc1:.2f}°" if enc1 is not None else "N/A"
            enc2_str = f"{enc2:.2f}°" if enc2 is not None else "N/A"
            
            prefix = ">>" if mid == motor_id else "  "
            status_lines.append(f"{prefix} ID {mid}: enc1={enc1_str}, enc2={enc2_str}")
            
        print("\n" + "\n".join(status_lines))
        
        cmd = input(f"正在标定 ID {motor_id}。按回车确认为零位，输入 s 跳过，输入 q 退出: ").strip().lower()
        if cmd == "":
            state = monitor.get_state(motor_id)
            v = state.get("enc2")
            if v is None:
                print("未读取到 enc2，重试")
                continue
            return float(v)
        if cmd == "s":
            return None
        if cmd == "q":
            raise KeyboardInterrupt()

def _apply_zero_and_lock(monitor, zero_map):
    tol_motor = config.STEER_ANGLE_TOLERANCE * config.STEER_REDUCTION_RATIO
    for mid, enc2_zero in zero_map.items():
        if enc2_zero is None:
            continue
        # 使用新接口更新参数 (同时更新 BasicConfig 和 Module)
        monitor.set_zero_calibration_params(mid, 0, float(enc2_zero))
        
        # 设置目标为 0 (适配 SwerveModule)
        if mid in monitor.id_to_module_map:
            mod, is_steer = monitor.id_to_module_map[mid]
            if is_steer:
                mod.target_angle = 0.0
        
    locked = set()
    t0 = time.time()
    while True:
        done = True
        for mid, enc2_zero in zero_map.items():
            if enc2_zero is None:
                continue
            state = monitor.get_state(mid)
            v = state.get("enc2")
            if v is None:
                done = False
                continue
            delta = ((v - enc2_zero + 180) % 360) - 180
            if abs(delta) <= tol_motor:
                locked.add(mid)
            else:
                done = False
        if done:
            break
        if time.time() - t0 > 30.0:
            break
        time.sleep(0.05)
    enc1_zero = {}
    for mid, enc2_zero in zero_map.items():
        if enc2_zero is None:
            continue
        state = monitor.get_state(mid)
        v1 = state.get("last_pos")
        if v1 is None:
            v1 = 0.0
        enc1_zero[mid] = float(v1)
    return enc1_zero, locked

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ids", type=str, default="", help="用逗号分隔的电机ID，留空使用默认转向ID")
    args = parser.parse_args()
    if args.ids:
        motor_ids = [int(x) for x in args.ids.split(",") if x.strip()]
    else:
        motor_ids = config.get_steer_ids()
    config.ENABLE_DRIVE = False
    # config.USE_CURRENT_AS_ZERO = False # Removed override to respect config
    monitor = VESCMonitor()
    monitor.enable_control = False # Disable control for manual calibration
    monitor.start()
    
    # 显式解锁电机
    time.sleep(0.5) # 等待连接建立
    _unlock_motors(monitor, motor_ids)
    
    ok = _wait_enc2_ready(monitor, motor_ids, 5.0)
    if not ok:
        print("enc2 未就绪")
    zeros = {}
    try:
        for mid in motor_ids:
            print(f"开始采样 ID {mid}，请物理回正到 0 度")
            v = _capture_enc2_zero(monitor, mid, all_ids=motor_ids)
            zeros[mid] = v
        print("应用零位并自动锁定")
        monitor.enable_control = True # Re-enable control for locking
        enc1_zero, locked = _apply_zero_and_lock(monitor, zeros)
        print("标定结果")
        print("STEER_ZERO_PARAMS 建议如下（使用 enc2 角度）：")
        out_items = []
        for mid in motor_ids:
            v = zeros.get(mid)
            if v is None:
                continue
            out_items.append(f"{mid}: (0, {v:.2f})")
        print("{ " + ", ".join(out_items) + " }")
        print("enc1 采样零位（用于切换后计角）：")
        for mid in motor_ids:
            v = enc1_zero.get(mid, 0.0)
            print(f"{mid}: {v:.2f}")
        if len(locked) != len([m for m in motor_ids if zeros.get(m) is not None]):
            print("部分电机未在容差内锁定")
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop()

if __name__ == "__main__":
    main()
