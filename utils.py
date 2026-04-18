import numpy as np
from . import EnemyInfo
from jsbsim import FGFDMExec

m2ft = lambda x: x / 0.3048  # 米转英尺
ft2m = lambda x: 0.3048 * x  # 英尺转米
const = lambda x, y=np.pi: np.mod(x + y, 2 * y) - y  # 把x约束到[-y,y]
# 距离,速度标准差正比于距离
sigma = lambda x: np.clip(x / 2e4, 0.03, 1) * np.array((100, 7))


class pid:  # 离散PID控制
    def __init__(self, dt, kp, ki, kd=0.0, limit=(-1, 1)):
        """
        dt:float,时间步长
        kp:float,比例增益,纠正强度
        ki:float,积分增益,惩罚累积误差
        kd:float,微分增益,提前刹车
        limit:tuple,输出限幅
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit = limit
        self.error = self.summa = 0

    def update(self, status, expect):
        """
        status:float,当前值
        expect:float,期望值
        """
        error = expect - status
        diff = (error - self.error) / self.dt
        self.error = error
        out = self.kp * error + self.ki * self.summa + self.kd * diff
        clip = np.clip(out, *self.limit)
        if self.ki and np.sign(out - clip) != np.sign(error):  # 不饱和才积分
            m, M = self.limit
            self.summa = np.clip(self.summa + error * self.dt, m / self.ki, M / self.ki)
        return clip


class model:
    ID = 1

    def __init__(self, root):
        # root:str,模型根目录,None则jsbsim
        base = type(self).mro()[-2]  # 避免子类ID副本
        self.ID = base.ID
        base.ID += 1
        self.fdm = FGFDMExec(root)
        self.fdm.set_debug_level(0)
        self.Longitude = self.Latitude = 0.0  # 经纬度(rad)
        self.Altitude = 0.0  # 高度(m)
        self.Yaw = self.Pitch = self.Roll = 0.0  # 航向,俯仰,滚转角(rad)
        self.p = self.q = self.r = 0.0  # 滚转,俯仰,偏航角速度(rad/s)
        self.vNED = np.ndarray(3)  # NED系速度向量(m/s)
        self.aNED = np.ndarray(3)  # NED系运动加速度向量(m/s^2)
        self.axyz = np.ndarray(3)  # 体轴加速度向量(m/s^2)
        self.RM = np.ndarray((3, 3))  # 体轴转NED的旋转矩阵
        self.Mach_M = 0.0  # 马赫
        self.fuel = 0.0  # 燃油(lbs)

    def rotate(self, flush=False):
        if flush:
            sy, sp, sr = map(np.sin, (self.Yaw, self.Pitch, self.Roll))
            cy, cp, cr = map(np.cos, (self.Yaw, self.Pitch, self.Roll))
            # 绕z轴转Yaw
            Rz = np.array(((cy, sy, 0), (-sy, cy, 0), (0, 0, 1)))
            # 绕y轴转Pitch
            Ry = np.array(((cp, 0, -sp), (0, 1, 0), (sp, 0, cp)))
            # 绕x轴转Roll
            Rx = np.array(((1, 0, 0), (0, cr, sr), (0, -sr, cr)))
            self.RM = Rz @ Ry @ Rx
        return self.RM

    def update(self):
        # 位置
        self.Latitude = self.fdm["position/lat-gc-rad"]
        self.Longitude = self.fdm["position/long-gc-rad"]
        self.Altitude = ft2m(self.fdm["position/h-sl-ft"])
        # 姿态
        self.Yaw = const(self.fdm["attitude/psi-rad"])
        self.Pitch = self.fdm["attitude/theta-rad"]
        self.Roll = self.fdm["attitude/phi-rad"]
        self.p = self.fdm["velocities/p-rad_sec"]
        self.q = self.fdm["velocities/q-rad_sec"]
        self.r = self.fdm["velocities/r-rad_sec"]
        # 速度
        self.vNED[0] = ft2m(self.fdm["velocities/v-north-fps"])
        self.vNED[1] = ft2m(self.fdm["velocities/v-east-fps"])
        self.vNED[2] = ft2m(self.fdm["velocities/v-down-fps"])
        self.Mach_M = self.fdm["velocities/mach"]
        self.fuel = self.fdm["propulsion/total-fuel-lbs"]
        # 加速度
        self.axyz[0] = ft2m(self.fdm["accelerations/a-pilot-x-ft_sec2"])
        self.axyz[1] = ft2m(self.fdm["accelerations/a-pilot-y-ft_sec2"])
        self.axyz[2] = ft2m(self.fdm["accelerations/a-pilot-z-ft_sec2"])
        self.aNED = self.rotate(True) @ self.axyz

    def ready(self, lat, lon, alt, mach, head, pitch, roll, fuel):
        """
        lat:float,初始纬度(deg)
        lon:初始经度
        alt:float,初始高度(m)
        mach:float,初始马赫
        head:float,初始方位角(deg)
        pitch:初始俯仰角
        roll:初始滚转角
        fuel:float,初始燃油(lbs)
        """
        if not self.fdm.get_model_name():
            raise AttributeError(f"{type(self)}.__init__未加载模型")
        self.fdm.set_dt(0.01)
        self.fdm["ic/lat-gc-deg"] = lat
        self.fdm["ic/long-gc-deg"] = lon
        self.fdm["ic/h-sl-ft"] = m2ft(alt)
        self.fdm["ic/mach"] = mach
        self.fdm["ic/psi-true-deg"] = head
        self.fdm["ic/theta-deg"] = pitch
        self.fdm["ic/phi-deg"] = roll
        self.fdm["propulsion/tank/contents-lbs"] = fuel
        self.fdm.run_ic()
        self.fdm["gear/gear-cmd-norm"] = 0  # 收起起落架
        self.fdm["propulsion/engine/set-running"] = 1  # 启动引擎
        self.update()

    def step1(self, aileron, elevator, throttle):
        """
        aileron:float,飞机归一化副翼(-1~1)
        elevator:归一化升降舵,+上-下
        throttle:float,归一化油门(0~1)
        """
        self.fdm["fcs/aileron-cmd-norm"] = np.clip(aileron, -1, 1)
        self.fdm["fcs/elevator-cmd-norm"] = np.clip(-elevator, -1, 1)  # 实则+下-上
        self.fdm["fcs/throttle-cmd-norm"] = np.clip(throttle, 0, 1)
        self.fdm["fcs/rudder-cmd-norm"] = 0
        gear = self.Altitude < ft2m(1500) and self.Mach_M < 0.5
        if gear != round(self.fdm["gear/gear-pos-norm"], 1):
            self.fdm["gear/gear-cmd-norm"] = gear
        self.fdm.run()
        self.update()

    def step2(self, CmdPhi, nz):
        """
        CmdPhi:float,导弹期望滚转角(rad)
        nz:float,期望法向过载(g)
        """
        self.fdm["fcs/phi-cmd-rad"] = np.clip(CmdPhi, -np.pi / 2, np.pi / 2)
        self.fdm["fcs/nz-cmd-norm"] = np.clip(nz, -40, 40)
        self.fdm["fcs/ny-cmd-norm"] = 0
        self.fdm["fcs/throttle-cmd-norm"] = 1
        self.fdm.run()
        self.update()

    def geo2NED(self, lat, lon, alt):  # 经纬高转NED向量(m)
        R = 6371e3 + self.Altitude
        r = R * np.cos(self.Latitude)
        lat -= self.Latitude
        lon -= self.Longitude
        return np.array((R * lat, r * lon, self.Altitude - alt))

    def NED2geo(self, vector):  # NED向量转经纬高
        R = 6371e3 + self.Altitude
        r = R * np.cos(self.Latitude)
        lat = self.Latitude + vector[0] / R
        lon = self.Longitude + vector[1] / r
        return lat, lon, self.Altitude - vector[2]

    def los2NED(self, yaw, pitch, dis):  # 视线角转NED向量
        yaw += self.Yaw
        pitch += self.Pitch
        N = np.cos(pitch) * np.cos(yaw)
        E = np.cos(pitch) * np.sin(yaw)
        D = -np.sin(pitch)
        return dis * np.array((N, E, D))

    def NED2los(self, vector):  # NED向量转视线角(rad)
        dis = float(np.linalg.norm(vector))
        if dis < 1:  # 避免近距离角度突变
            return 0, 0, 1
        yaw = const(np.arctan2(vector[1], vector[0]) - self.Yaw)
        pitch = np.arcsin(-vector[2] / dis) - self.Pitch
        return yaw, pitch, dis

    def find(self, enemy, pos=np.zeros(3)):
        if not pos.any():
            pos = self.geo2NED(enemy.Latitude, enemy.Longitude, enemy.Altitude)
        # 距离,速度标准差
        std = sigma(np.linalg.norm(pos))
        # 带噪距离
        pos = np.random.normal(pos, std[0])
        info = EnemyInfo(enemy.ID)
        # 带噪速度
        info.vNED = np.random.normal(enemy.vNED, std[1])
        # 带噪声计算其他
        info.TargetYaw, info.TargetPitch, info.TargetDis = self.NED2los(pos)  # 极坐标
        # 径向相对速度
        info.DisRate = np.dot(info.vNED - self.vNED, pos) / info.TargetDis
        # 测量马赫
        info.TargetMach_M = float(np.linalg.norm(info.vNED)) / 303.77  # 9km声速
        return info, pos, std
