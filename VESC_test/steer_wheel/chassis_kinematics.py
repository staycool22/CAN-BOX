import numpy as np
import math
from dataclasses import dataclass
from typing import List, Dict, Tuple

@dataclass
class WheelState:
    """单个舵轮的状态"""
    speed_mps: float  # 轮线速度 (m/s)
    angle_rad: float  # 舵角 (rad), 0为正前方, 逆时针为正

class ChassisGeometry:
    def __init__(self, length: float = 0.6, width: float = 0.5, wheel_radius: float = 0.075):
        """
        :param length: 前后轮轴距 (m)
        :param width: 左右轮距 (m)
        :param wheel_radius: 轮胎半径 (m)
        """
        self.L = length
        self.W = width
        self.R = wheel_radius
        
        # 定义四个轮子的位置 (x, y)
        # 坐标系: 车体中心为原点, X轴向前, Y轴向左
        self.wheel_positions = {
            "FL": (self.L / 2, self.W / 2),
            "FR": (self.L / 2, -self.W / 2),
            "RL": (-self.L / 2, self.W / 2),
            "RR": (-self.L / 2, -self.W / 2)
        }

class FourWheelSteeringKinematics:
    def __init__(self, geometry: ChassisGeometry):
        self.geo = geometry

    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> Dict[str, Tuple[float, float]]:
        """
        逆运动学: 只有底盘目标速度 -> 计算各轮目标速度和角度
        :param vx: X轴速度 (m/s)
        :param vy: Y轴速度 (m/s)
        :param omega: 角速度 (rad/s)
        :return: 字典 {wheel_key: (speed_mps, angle_rad)}
        """
        results = {}
        for name, (wx, wy) in self.geo.wheel_positions.items():
            # 计算轮子处的速度矢量
            # v_wheel = v_chassis + omega x r
            # 注意: 这里为了匹配底盘实际旋转方向，反转了 omega 的方向
            v_wx = vx + omega * wy
            v_wy = vy - omega * wx
            
            speed = math.sqrt(v_wx**2 + v_wy**2)
            if speed < 1e-4:
                angle = 0.0
            else:
                # 标准 atan2 计算角度，0度为正前方 (X轴)
                angle_rad = math.atan2(v_wy, v_wx)
                
                # 归一化到 -pi ~ pi
                # angle_rad = (angle_rad + math.pi) % (2 * math.pi) - math.pi
                
                # 转换为度 (degrees)
                angle_deg = math.degrees(angle_rad)
                
                # 归一化到 0 ~ 360 度
                angle = angle_deg % 360.0
            
            results[name] = (speed, angle)
            
        return results

    def forward_kinematics(self, wheel_states: Dict[str, WheelState]) -> Tuple[float, float, float]:
        """
        正运动学 (里程计): 已知各轮状态 -> 估计底盘速度
        使用最小二乘法求解超定方程组
        :param wheel_states: 字典 {wheel_key: WheelState}
        :return: (vx, vy, omega)
        """
        # 构建方程组 A * [vx, vy, omega]^T = b
        # 对于每个轮子 i:
        # vx - yi * omega = vi * cos(theta_i)
        # vy + xi * omega = vi * sin(theta_i)
        
        A_rows = []
        b_rows = []
        
        for name, pos in self.geo.wheel_positions.items():
            if name not in wheel_states:
                continue
                
            state = wheel_states[name]
            xi, yi = pos
            vi = state.speed_mps
            theta = state.angle_rad
            
            # X方向方程
            A_rows.append([1, 0, -yi])
            b_rows.append(vi * math.cos(theta))
            
            # Y方向方程
            A_rows.append([0, 1, xi])
            b_rows.append(vi * math.sin(theta))
            
        if not A_rows:
            return 0.0, 0.0, 0.0
            
        A = np.array(A_rows)
        b = np.array(b_rows)
        
        # 最小二乘求解
        # x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        # 为了更明确的控制，使用 pseudo-inverse
        try:
            x = np.linalg.pinv(A) @ b
            return float(x[0]), float(x[1]), float(x[2])
        except np.linalg.LinAlgError:
            return 0.0, 0.0, 0.0

class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        
    def update(self, vx: float, vy: float, omega: float, current_time: float):
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 在世界坐标系中积分
        # 假设 vx, vy 是车体坐标系下的速度
        # 需要旋转到世界坐标系
        
        delta_x_robot = vx * dt
        delta_y_robot = vy * dt
        delta_theta = omega * dt
        
        sin_theta = math.sin(self.theta)
        cos_theta = math.cos(self.theta)
        
        # 旋转变换
        delta_x_world = delta_x_robot * cos_theta - delta_y_robot * sin_theta
        delta_y_world = delta_x_robot * sin_theta + delta_y_robot * cos_theta
        
        self.x += delta_x_world
        self.y += delta_y_world
        self.theta += delta_theta
        
        # 归一化角度到 -pi ~ pi
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def get_pose(self):
        return self.x, self.y, self.theta
