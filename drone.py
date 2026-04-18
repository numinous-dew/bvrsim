from . import DroneInfo, SendData
from .utils import *
from .missile import aim120c
from collections import defaultdict

table = (
    (np.linspace(0, 18e3, 7), (0.35, 0.5, 0.65, 0.8, 1, 1.15, 1.25)),  # 高度(m)
    (np.linspace(0.5, 1.5, 5), (0.7, 0.9, 1, 1.15, 1.25)),  # 马赫
    (np.linspace(0, np.pi, 7), (1.3, 1.2, 1, 0.8, 0.6, 0.45, 0.35)),  # 相对方位角(rad)
)
MissileTrackList = defaultdict(list[aim120c])  # 已射导弹字典


class f16(model):
    def __init__(self, lat, lon, alt, mach, num, head, pitch=0.0, roll=0.0, fuel=5e3):
        super().__init__(None)
        self.fdm.load_model("f16")
        self.radarR = 14e4  # 雷达半径(m)
        self.radarAngle = np.deg2rad(60)  # 雷达角度(rad)
        # 滚转角->副翼
        self.roll2ail = pid(self.fdm.get_delta_t(), 4, 0, 0.15)
        # 高度->俯仰角->升降舵
        self.alt2pitch = pid(self.fdm.get_delta_t(), 0.2625, 0.005)
        self.pitch2ele = pid(self.fdm.get_delta_t(), 3.5, 0, 0.25)
        # 马赫->油门
        self.mach2thr = pid(self.fdm.get_delta_t(), 2, 0.15)
        self.AlarmList = []
        self.FoundEnemyList = []  # EnemyInfo
        self.MissileNowNum = num
        self.engage = 0
        super().ready(lat, lon, alt, mach, head, pitch, roll, fuel)

    def step(self, cmd: SendData):
        enemy = next((x for x in self.FoundEnemyList if x.EnemyID == cmd.EnemyID), None)
        if not self.engage and enemy:
            if -1 == cmd.engage:  # 火控锁定
                enemy.isNTS = True
            elif (
                1 == cmd.engage
                and self.MissileNowNum  # 有导弹
                and enemy.isNTS  # 已锁定
                and enemy.TargetDis <= enemy.MissileMaxDis  # 距离满足
            ):  # 发射导弹
                self.MissileNowNum -= 1
                MissileTrackList[self.ID].append(aim120c(self, enemy))
        self.engage = cmd.engage
        # 相对方位角
        CmdHeadingDeg = const(cmd.CmdHeadingDeg - np.rad2deg(self.Yaw), 180)
        # 是否就远转
        if np.sign(cmd.TurnDirection * CmdHeadingDeg) < 0:
            CmdHeadingDeg = cmd.TurnDirection * (360 - abs(CmdHeadingDeg))
        # 最大滚转角
        CmdPhi = min(np.pi / 2, np.deg2rad(abs(cmd.CmdPhi)))
        # 1:1期望滚转角
        CmdPhi = np.clip(CmdHeadingDeg, -CmdPhi, CmdPhi)
        # 副翼
        aileron = self.roll2ail.update(self.Roll, CmdPhi)
        # 最大俯仰角
        CmdPitchDeg = min(np.pi / 2, np.deg2rad(abs(cmd.CmdPitchDeg)))
        self.alt2pitch.limit = -CmdPitchDeg, CmdPitchDeg
        # 期望俯仰角
        CmdPitchDeg = self.alt2pitch.update(self.Altitude, cmd.CmdAlt)
        # 升降舵
        elevator = self.pitch2ele.update(self.Pitch, CmdPitchDeg)
        # 油门限幅
        self.mach2thr.limit = 0.1, np.clip(cmd.ThrustLimit / 129, 0.1, 1)
        # 油门
        throttle = self.mach2thr.update(self.Mach_M, cmd.CmdSpd)
        super().step1(aileron, elevator, throttle)

    def alarm(self, AlarmID, MisAzi, AlarmType):
        """
        AlarmID:int,辐射源ID
        MisAzi:float,相对方位角(rad)
        AlarmType:str,告警类型
        """
        self.AlarmList.append((AlarmID, MisAzi, AlarmType))

    def find(self, info: EnemyInfo, pos, std, isNTS):
        info.isNTS = isNTS
        for (xp, fp), x in zip(table[:2], (self.Altitude, self.Mach_M)):
            x = np.interp(x, xp, fp)
            info.MissilePowerfulDis *= x
            info.MissileMaxDis *= x
        # 马赫反比于NEZ
        info.MissilePowerfulDis *= max(0.5, 1 - info.TargetMach_M / 10)
        # 攻角影响射程
        info.MissileMaxDis *= np.interp(abs(info.TargetYaw), *table[2])
        self.FoundEnemyList.append(info)
        # 导弹数据链
        for misl in MissileTrackList[self.ID]:
            if info.EnemyID == misl.TargetID and misl.state < 2:
                # 载机到导弹的相对坐标
                rel = super().geo2NED(misl.Latitude, misl.Longitude, misl.Altitude)
                # info只要vNED
                misl.find(info, pos - rel, std)

    def getinfo(self):
        info = DroneInfo(self)
        info.strike = [x.TargetID for x in MissileTrackList[self.ID] if x.state == 2]
        return info
