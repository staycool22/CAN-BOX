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

    def inverse_kinematics(self, vx: float, vy: float, omega: float, optimize_angle: bool = True) -> Dict[str, Tuple[float, float]]:
        """
        逆运动学: 只有底盘目标速度 -> 计算各轮目标速度和角度
        :param vx: X轴速度 (m/s)
        :param vy: Y轴速度 (m/s)
        :param omega: 角速度 (rad/s)
        :param optimize_angle: 是否启用舵轮角度优化 (就近转动 + 反转速度)
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
                angle_deg = math.degrees(angle_rad)
                
                # --- 舵轮角度优化 (Swerve Optimization) ---
                if optimize_angle:
                    # 确保角度在 -90 ~ 90 度 (或 270 ~ 90) 范围内
                    # 如果角度在背侧 (90 ~ 270 或 -270 ~ -90)，则翻转 180 度并反转速度
                    # 归一化到 -180 ~ 180
                    angle_norm = (angle_deg + 180) % 360 - 180
                    
                    if angle_norm > 90:
                        angle_norm -= 180
                        speed = -speed
                    elif angle_norm < -90:
                        angle_norm += 180
                        speed = -speed
                    
                    # 最终输出 0-360 或 -180-180 均可，这里保持 0-360 格式以便兼容
                    angle = angle_norm % 360.0
                else:
                    # 不优化，直接输出原始角度 (0-360)
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

class AckermannSteeringKinematics:
    """
    阿克曼转向运动学 (2WS 或 4WS)
    """
    def __init__(self, geometry: ChassisGeometry, is_4ws: bool = False, max_steer_angle_deg: float = 45.0):
        self.geo = geometry
        self.is_4ws = is_4ws
        self.max_steer_angle = max_steer_angle_deg

    def inverse_kinematics(self, vx: float, vy: float, omega: float, optimize_angle: bool = True) -> Dict[str, Tuple[float, float]]:
        """
        阿克曼逆运动学
        :param vx: 纵向速度 (m/s)
        :param vy: 横向速度 (m/s) - 在阿克曼模式下通常被忽略或作为错误
        :param omega: 横摆角速度 (rad/s)
        :param optimize_angle: 是否启用舵轮角度优化 (就近转动 + 反转速度)
        """
        results = {}
        
        # 避免除以零
        if abs(omega) < 1e-5:
            # 直行
            for name in self.geo.wheel_positions:
                results[name] = (vx, 0.0)
            return results

        # 1. 确定旋转中心 (ICR)
        # 对于 4WS (对称): ICR 在 Y 轴上 (x=0), R = vx / omega
        # 对于 2WS (前轮): ICR 在后轴延长线上 (x=-L/2), R_rear = vx / omega
        
        # 为了统一接口，我们假设输入的 vx, omega 是针对【底盘几何中心】的期望值
        R_center = vx / omega
        
        for name, (wx, wy) in self.geo.wheel_positions.items():
            is_front = (wx > 0)
            
            if self.is_4ws:
                # --- 4WS 双阿克曼 (反相) ---
                # 旋转中心在 Y 轴 (x=0, y=R_center)
                # 轮子位置 (wx, wy), ICR (0, R_center)
                # 向量 ICR->Wheel = (wx, wy - R_center)
                # 轮子速度方向垂直于此向量
                
                dx = wx - 0
                dy = wy - R_center
                
                # 计算轮子处的线速度 V = Omega * Distance
                dist = math.sqrt(dx**2 + dy**2)
                speed = omega * dist
                                
                # 直接使用通用公式即可完美实现 4WS Ackermann
                v_wx = vx + omega * wy
                v_wy = -omega * wx # vy=0
                
                speed = math.sqrt(v_wx**2 + v_wy**2)
                angle_rad = math.atan2(v_wy, v_wx)
                angle = math.degrees(angle_rad)
                
            else:
                # --- 2WS 前轮转向 ---
                # 假设旋转中心在后轴 (x = -L/2)
                # 此时底盘中心的侧滑角不为0，或者说 vx, omega 定义在后轴中心更合适
                # 但为了兼容，我们重新计算基于后轴中心的 R
                
                # 转移速度到后轴中心:
                # v_rear = vx - omega * (W_center_to_rear_y?) No, simple translation.
                # v_rear_x = vx
                # v_rear_y = vy - omega * (L/2) = -omega * L/2
                # 实际上 2WS 的非完整约束要求 v_rear_y = 0 (无侧滑)
                # 这意味着我们不能随意指定中心的 (vx, vy=0, omega)。
                # 如果 vy=0，则 omega 必须为 0 才能满足 2WS 无侧滑约束 (除非是在打滑)。
                # 
                # 所以，通常 Ackermann 控制输入是 (Speed, SteeringAngle)。
                # 这里我们假设输入的 omega 是【期望的转向率】，并根据它计算所需的 SteeringAngle。
                # 忽略中心侧滑约束，强制后轮为0，前轮转向以匹配 omega。
                
                if not is_front:
                    # 后轮锁定
                    speed = vx + omega * wy # 差速
                    angle = 0.0
                else:
                    # 前轮: 基于几何计算
                    # ICR 坐标: (-L/2, R_rear)
                    # R_rear = vx / omega (近似)
                    
                    # 使用通用公式计算前轮，但基于 ICR 在后轴的假设？
                    # 不，直接用通用公式计算前轮会导致后轮也被要求转向 (如果用通用公式算后轮)。
                    # 但对于前轮，通用公式给出的角度是“为了让该点产生(vx, -omega*x)速度”所需的角度。
                    # 如果我们只应用给前轮，前轮会产生正确的横向力矩。
                    
                    # 让我们用通用公式计算前轮状态
                    v_wx = vx + omega * wy
                    v_wy = -omega * wx
                    
                    speed = math.sqrt(v_wx**2 + v_wy**2)
                    angle_rad = math.atan2(v_wy, v_wx)
                    angle = math.degrees(angle_rad)
            
            # 统一优化角度 (Swerve Optimization)
            # 即便在 Ackermann 模式，优化也是有用的 (例如倒车时)
            angle_norm = (angle + 180) % 360 - 180
            
            # --- 阿克曼模式下的限位保护 ---
            # 在这里直接截断角度，防止出现 90 度平移
            if angle_norm > self.max_steer_angle:
                angle_norm = self.max_steer_angle
            elif angle_norm < -self.max_steer_angle:
                angle_norm = -self.max_steer_angle
            
            # 注意: 如果不需要倒车优化 (Swerve Optimization)，上面的限位就已经够了。
            # 但如果我们仍希望支持倒车 (例如 angle=170 -> angle=-10, speed=-speed)，
            # 则应该先做 Swerve Optimization，再做限位。
            # 这里的逻辑顺序是: 
            # 1. atan2 算出原始角度 (-180~180)
            # 2. 如果原始角度在后方 (例如 170)，翻转为前方 (-10)，速度反向
            # 3. 限制前方角度在 +/- 45 度内
            
            # 重新整理逻辑:
            raw_angle = angle
            raw_speed = speed
            
            # 1. 归一化到 -180 ~ 180
            angle_norm = (raw_angle + 180) % 360 - 180
            
            # 2. Swerve Optimization (翻转到前半球 -90 ~ 90)
            if optimize_angle:
                if angle_norm > 90:
                    angle_norm -= 180
                    raw_speed = -raw_speed
                elif angle_norm < -90:
                    angle_norm += 180
                    raw_speed = -raw_speed
                
            # 3. Ackermann Angle Limit (限制在 -45 ~ 45)
            if angle_norm > self.max_steer_angle:
                angle_norm = self.max_steer_angle
            elif angle_norm < -self.max_steer_angle:
                angle_norm = -self.max_steer_angle
            
            # 2WS 模式下强制后轮归零 (覆盖上面的通用计算)
            if not self.is_4ws and not is_front:
                angle_norm = 0.0
                # Speed 仍保留差速 (v_wx)
                raw_speed = vx + omega * wy
            
            results[name] = (raw_speed, angle_norm)
            
        return results
