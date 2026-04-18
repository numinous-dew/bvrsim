from .drone import *
from glob import glob
from datetime import datetime


class tacview:
    def __init__(self, exist_ok):
        # exist_ok:bool,是否覆盖原文件
        file = f"tacview_{len(glob('tacview*.acmi'))+(not exist_ok)or 1}.acmi"
        self.file = open(file, "w", encoding="utf-8")
        self.file.write("FileType=text/acmi/tacview\nFileVersion=2.1\n0,ReferenceTime=")
        self.file.write(datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ") + "\n")

    def __del__(self):
        self.file.close()

    def logtime(self, time):
        self.file.write(f"#{time:.3f}\n")

    def loginit(self, entity: f16 | aim120c, color, name="F-16C", radar=1):
        deg = np.rad2deg(entity.radarAngle)
        self.file.write(
            f"{entity.ID:X},Color={color},Name={name},RadarMode={radar},RadarAzimuth={deg},RadarElevation={deg},RadarRange={entity.radarR},Radius=5\n"
        )

    def logstep(self, entity: f16 | aim120c):  # or AIM-120C
        lon, lat, roll, pitch, yaw = np.rad2deg(
            (entity.Longitude, entity.Latitude, entity.Roll, entity.Pitch, entity.Yaw)
        )
        self.file.write(
            f"{entity.ID:X},T={lon:.6f}|{lat:.6f}|{entity.Altitude:.2f}|{roll:.2f}|{pitch:.2f}|{yaw:.2f}\n"
        )

    def logNTS(self, entityID, TargetID):
        self.file.write(f"{entityID:X},LockedTargetMode=1,LockedTarget={TargetID:X}\n")

    def logdestroy(self, TargetID):
        self.file.write(f"-{TargetID:X}\n")
