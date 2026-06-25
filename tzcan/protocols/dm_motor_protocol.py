from time import sleep
from enum import IntEnum
from struct import pack
from struct import unpack

import numpy as np

from .base import CANProtocolBase


class Motor:
    def __init__(self, MotorType, SlaveID, MasterID):
        self.Pd = float(0)
        self.Vd = float(0)
        self.state_q = float(0)
        self.state_dq = float(0)
        self.state_tau = float(0)
        self.state_err = int(0)
        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType
        self.isEnable = False
        self.NowControlMode = Control_Type.MIT
        self.temp_param_dict = {}

    def recv_data(self, q: float, dq: float, tau: float, err: int):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau
        self.state_err = err

    def getPosition(self):
        return self.state_q

    def getVelocity(self):
        return self.state_dq

    def getTorque(self):
        return self.state_tau

    def getError(self):
        return self.state_err

    def getParam(self, RID):
        if RID in self.temp_param_dict:
            return self.temp_param_dict[RID]
        else:
            return None


class DM_MOTOR(CANProtocolBase):
    Limit_Param = [
        [12.5, 30, 10],
        [12.5, 50, 10],
        [12.5, 10, 28],
        [12.5, 10, 28],
        [12.5, 45, 20],
        [12.5, 45, 40],
        [12.5, 45, 54],
        [12.5, 25, 200],
        [12.5, 20, 200],
        [12.5, 280, 1],
        [12.5, 45, 10],
        [12.5, 45, 10],
        [12.5, 10, 12],
        [12.566, 20, 120],
        [12.566, 50, 5],
    ]

    def __init__(self, transmitter):
        super().__init__(transmitter)
        self.motors_map = dict()

    def controlMIT(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        if DM_Motor.SlaveID not in self.motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX = self.Limit_Param[MotorType][0]
        DQ_MAX = self.Limit_Param[MotorType][1]
        TAU_MAX = self.Limit_Param[MotorType][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xFF
        data_buf[1] = q_uint & 0xFF
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
        data_buf[4] = kp_uint & 0xFF
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
        data_buf[7] = tau_uint & 0xFF
        self.__send_data(DM_Motor.SlaveID, data_buf)
        self.recv()

    def control_delay(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float, delay: float):
        self.controlMIT(DM_Motor, kp, kd, q, dq, tau)
        sleep(delay)

    def control_Pos_Vel(self, Motor, P_desired: float, V_desired: float):
        if Motor.SlaveID not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        sleep(0.001)
        self.recv()

    def control_Vel(self, Motor, Vel_desired):
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()

    def control_pos_force(self, Motor, Pos_des: float, Vel_des, i_des):
        if Motor.SlaveID not in self.motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(Pos_des)
        data_buf[0:4] = Pos_desired_uint8s
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xFF
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xFF
        data_buf[7] = ides_uint >> 8
        self.__send_data(motorid, data_buf)
        self.recv()

    def control_Pos_Vel_CSP(self, Motor, P_desired: float, V_desired: float):
        if Motor.SlaveID not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x400 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()

    def control_Vel_CSP(self, Motor, Vel_desired):
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x500 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()

    def control_Tor_CSP(self, Motor, Tor_desired):
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x600 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Tor_desired_uint8s = float_to_uint8s(Tor_desired)
        data_buf[0:4] = Tor_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()

    def enable(self, Motor):
        self.__control_cmd(Motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()

    def enable_old(self, Motor, ControlMode):
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], np.uint8)
        enable_id = ((int(ControlMode) - 1) << 2) + Motor.SlaveID
        self.__send_data(enable_id, data_buf)
        sleep(0.1)
        self.recv()

    def disable(self, Motor):
        self.__control_cmd(Motor, np.uint8(0xFD))
        sleep(0.01)

    def set_zero_position(self, Motor):
        self.__control_cmd(Motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()

    def recv(self, timeout: float = 0):
        while True:
            CANID, data = self.receive(_is_ext_frame=False, _is_fd=False, timeout=timeout)
            if CANID is None or data is None:
                return
            self.__process_packet(data, CANID)
            timeout = 0

    def recv_set_param_data(self, timeout: float = 0):
        while True:
            CANID, data = self.receive(_is_ext_frame=False, _is_fd=False, timeout=timeout)
            if CANID is None or data is None:
                return
            self.__process_set_param_packet(data, CANID)
            timeout = 0

    def __process_packet(self, data, CANID):
        if CANID != 0x00:
            if CANID in self.motors_map:
                err_int = int((np.uint8(data[0]) >> 4) & 0x0F)
                q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                tau_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
                MotorType_recv = self.motors_map[CANID].MotorType
                Q_MAX = self.Limit_Param[MotorType_recv][0]
                DQ_MAX = self.Limit_Param[MotorType_recv][1]
                TAU_MAX = self.Limit_Param[MotorType_recv][2]
                recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau, err_int)
        else:
            MasterID = data[0] & 0x0F
            if MasterID in self.motors_map:
                err_int = int((np.uint8(data[0]) >> 4) & 0x0F)
                q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                tau_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
                MotorType_recv = self.motors_map[MasterID].MotorType
                Q_MAX = self.Limit_Param[MotorType_recv][0]
                DQ_MAX = self.Limit_Param[MotorType_recv][1]
                TAU_MAX = self.Limit_Param[MotorType_recv][2]
                recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                self.motors_map[MasterID].recv_data(recv_q, recv_dq, recv_tau, err_int)

    def __process_set_param_packet(self, data, CANID):
        if data[2] == 0x33 or data[2] == 0x55:
            masterid = CANID
            slaveId = (data[1] << 8) | data[0]
            if CANID == 0x00:
                masterid = slaveId

            if masterid not in self.motors_map:
                if slaveId not in self.motors_map:
                    return
                else:
                    masterid = slaveId

            RID = data[3]
            if is_in_ranges(RID):
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num
            else:
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num

    def addMotor(self, Motor):
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor
        return True

    def __control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd], np.uint8)
        self.__send_data(Motor.SlaveID, data_buf)

    def __send_data(self, motor_id, data):
        return self.send(int(motor_id), list(data), _is_ext_frame=False, _is_fd=False)

    def __read_RID_param(self, Motor, RID):
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x33, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)

    def __write_motor_param(self, Motor, RID, data):
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x55, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        if not is_in_ranges(RID):
            data_buf[4:8] = float_to_uint8s(data)
        else:
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switchControlMode(self, Motor, ControlMode):
        max_retries = 10
        retry_interval = 0.05
        RID = 10
        self.__write_motor_param(Motor, RID, np.uint8(ControlMode))
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    if self.motors_map[Motor.SlaveID].temp_param_dict[RID] == ControlMode:
                        return True
                    else:
                        return False
        return False

    def save_motor_param(self, Motor):
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.disable(Motor)
        self.__send_data(0x7FF, data_buf)
        sleep(0.001)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def refresh_motor_status(self, Motor):
        can_id_l = Motor.SlaveID & 0xFF
        can_id_h = (Motor.SlaveID >> 8) & 0xFF
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)
        self.recv()

    def change_motor_param(self, Motor, RID, data):
        max_retries = 20
        retry_interval = 0.05

        self.__write_motor_param(Motor, RID, data)
        for _ in range(max_retries):
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map and RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                if abs(self.motors_map[Motor.SlaveID].temp_param_dict[RID] - data) < 0.1:
                    return True
                else:
                    return False
            sleep(retry_interval)
        return False

    def read_motor_param(self, Motor, RID):
        max_retries = 20
        retry_interval = 0.05
        self.__read_RID_param(Motor, RID)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    return self.motors_map[Motor.SlaveID].temp_param_dict[RID]
                else:
                    return None
        return None


def LIMIT_MIN_MAX(x, min, max):
    if x <= min:
        return min
    elif x > max:
        return max
    return x


def float_to_uint(x: float, x_min: float, x_max: float, bits):
    x = LIMIT_MIN_MAX(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


def uint_to_float(x: np.uint16, min: float, max: float, bits):
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)


def float_to_uint8s(value):
    packed = pack("f", value)
    return unpack("4B", packed)


def data_to_uint8s(value):
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        packed = pack("I", value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")
    return unpack("4B", packed)


def is_in_ranges(number):
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False


def uint8s_to_uint32(byte1, byte2, byte3, byte4):
    packed = pack("<4B", byte1, byte2, byte3, byte4)
    return unpack("<I", packed)[0]


def uint8s_to_float(byte1, byte2, byte3, byte4):
    packed = pack("<4B", byte1, byte2, byte3, byte4)
    return unpack("<f", packed)[0]


def print_hex(data):
    hex_values = [f"{byte:02X}" for byte in data]
    print(" ".join(hex_values))


def get_enum_by_index(index, enum_class):
    try:
        return enum_class(index)
    except ValueError:
        return None


class DM_Motor_Type(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM4340_48V = 3
    DM6006 = 4
    DM8006 = 5
    DM8009 = 6
    DM10010L = 7
    DM10010 = 8
    DMH3510 = 9
    DMH6215 = 10
    DMG6220 = 11
    DMJH11 = 12
    DM6248P = 13
    DM3507 = 14


class DM_variable(IntEnum):
    UV_Value = 0
    KT_Value = 1
    OT_Value = 2
    OC_Value = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    Damp = 11
    Inertia = 12
    hw_ver = 13
    sw_ver = 14
    SN = 15
    NPP = 16
    Rs = 17
    LS = 18
    Flux = 19
    Gr = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_Value = 29
    GREF = 30
    Deta = 31
    V_BW = 32
    IQ_c1 = 33
    VL_c1 = 34
    can_br = 35
    sub_ver = 36
    u_off = 50
    v_off = 51
    k1 = 52
    k2 = 53
    m_off = 54
    dir = 55
    p_m = 80
    xout = 81


class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    Torque_Pos = 4
    POS_VEL_CSP = 5
    VEL_CSP = 6
    Torque_CSP = 7
