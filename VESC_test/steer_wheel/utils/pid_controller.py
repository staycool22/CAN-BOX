import time

class PIDController:
    """
    通用 PID 控制器
    """
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, error, dt=None):
        """
        计算 PID 输出
        :param error: 当前误差
        :param dt: 时间间隔 (秒)。如果为 None，则自动计算。
        :return: 控制输出
        """
        current_time = time.time()
        
        if dt is None:
            if self.last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.last_time
        
        self.last_time = current_time
        
        # 积分项
        # 注意：积分限幅由外部实现（根据用户要求），这里仅做累加
        if dt > 0:
            self.integral += error * dt
            
        # 微分项
        derivative = 0.0
        if dt > 0:
            derivative = (error - self.prev_error) / dt
            
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        return output

    def reset(self):
        """重置内部状态"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None
