import math

def normalize_angle(angle_deg):
    """将角度归一化到 -180 ~ 180 度"""
    return (angle_deg + 180) % 360 - 180

def optimize_swerve_angle(target_angle, current_angle):
    """
    舵轮角度优化：计算最短路径，避免不必要的 360 度旋转。
    返回: (目标绝对角度, 是否需要反转速度)
    """
    # 计算目标角度与当前角度的差值，并归一化到 -180 ~ 180
    diff = normalize_angle(target_angle - current_angle)
    
    # 如果旋转角度大于 90 度，则翻转 180 度并反转电机速度
    # 这样轮子只需要转动较小的角度（<90度）
    if diff > 90:
        return current_angle + (diff - 180), True
    elif diff < -90:
        return current_angle + (diff + 180), True
    else:
        return current_angle + diff, False

def get_shortest_path_to_angle(target_angle, current_angle):
    """
    计算到达目标角度的最短路径（不进行 180 度翻转优化）。
    仅旋转到最近的同相角度。
    用于回零校准等强制角度对齐的场景。
    """
    diff = normalize_angle(target_angle - current_angle)
    return current_angle + diff

def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)
