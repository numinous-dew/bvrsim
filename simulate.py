import multiprocessing as mu
from .drone import *
from .tacview import tacview


def inradar(entity: f16 | aim120c, info: EnemyInfo):
    return (  # 角度条件
        np.cos(info.TargetYaw) * np.cos(info.TargetPitch) > np.cos(entity.radarAngle)
        and info.TargetDis < entity.radarR  # 距离条件
    )


class spatialgrid:  # 均匀网格优化邻近点查找
    def __init__(self, size):
        self.size = size
        self.grid = defaultdict(list)

    def clear(self):
        self.grid.clear()

    def hash(self, pos: np.ndarray):
        return tuple((pos // self.size).astype(int))

    def add(self, ID, pos):
        self.grid[self.hash(pos)].append(ID)

    def getnear(self, pos):
        key = self.hash(pos)
        near = []
        for i in range(key[0] - 1, key[0] + 2):
            for j in range(key[1] - 1, key[1] + 2):
                for k in range(key[2] - 1, key[2] + 2):
                    near.extend(self.grid[(i, j, k)])
        return near


class bvrsim:
    def __init__(
        self,
        field=((23, 26), (118, 120), (2e3, 15e3)),  # tuple,战场纬,经,高度范围(deg,m)
        threat=((24.5, 119, 0, 5e4),),  # tuple,红方威胁区中心纬,经度(deg),高度,半径(m)
    ):
        self.field = *np.deg2rad(field[:2]), field[2]
        # 战场中心(原点)
        self.center = np.mean(self.field, 1)
        self.threat = [np.append(np.deg2rad(x[:3]), x[3]) for x in threat]
        # 红方初始条件
        self.red = (
            dict(lat=25.8, lon=118.2, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
            dict(lat=25.8, lon=118.4, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
            dict(lat=25.8, lon=118.6, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
            dict(lat=25.8, lon=118.8, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
        )
        self.blue = (  # 蓝方初始条件
            dict(lat=23.2, lon=118.2, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
            dict(lat=23.2, lon=118.4, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
            dict(lat=23.2, lon=118.6, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
            dict(lat=23.2, lon=118.8, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
        )

    def redstrategy(self, info: DroneInfo, step_num):  # 红方策略函数
        cmd = SendData()
        cmd.CmdSpd = 2
        cmd.CmdAlt = 13e3
        expect = 23.5, 119
        if 24 < info.Latitude:
            if info.DroneID == 1:
                expect = [24, 118.15]
            elif info.DroneID == 2:
                expect = [24, 118.35]
            elif info.DroneID == 3:
                expect = [24, 119.65]
            else:
                expect = [24, 119.85]
            expect[0] += 25 < info.Latitude
        R = 6371e3 + info.Altitude
        r = R * np.cos(info.Latitude)
        expect = np.deg2rad(expect) - (info.Latitude, info.Longitude)
        expect *= (R, r)
        cmd.CmdHeadingDeg = np.rad2deg(np.arctan2(expect[1], expect[0]))
        if not step_num % 200:
            cmd.engage = -1
        elif step_num % 200 == 100 and 10 <= np.rad2deg(info.Pitch):  # 保证发射仰角
            cmd.engage = 1
        cmd.EnemyID = info.DroneID + 4
        return cmd

    def bluestrategy(self, info: DroneInfo, step_num):  # 蓝方策略函数
        cmd = SendData()
        cmd.CmdSpd = 1.2
        cmd.CmdAlt = 11e3
        if not step_num % 200:
            cmd.engage = -1
        elif step_num % 200 == 100 and 10 <= np.rad2deg(info.Pitch):
            cmd.engage = 1
        cmd.EnemyID = info.DroneID - 4
        return cmd

    def redconsumer(self, get: mu.Queue, put: mu.Queue):  # 红方消费者
        while True:
            data = get.get()
            if not data:
                break
            put.put((data[0].DroneID, self.redstrategy(*data)))

    def blueconsumer(self, get: mu.Queue, put: mu.Queue):  # 蓝方消费者
        while True:
            data = get.get()
            if not data:
                break
            put.put((data[0].DroneID, self.bluestrategy(*data)))

    def restrict(self):  # 检查是否有效
        dead = set()
        for ID, (entity, color) in self.entity.items():
            if type(entity) == f16 and entity.fuel < 1:  # 飞机无燃料
                dead.add(ID)
                continue
            for (m, M), x in zip(
                self.field, (entity.Latitude, entity.Longitude, entity.Altitude)
            ):
                if x < m or M < x:  # 飞出战场
                    dead.add(ID)
                    break
            if "Red" == color and not ID in dead:
                for lat, lon, alt, r in self.threat:
                    # 是否飞入威胁区
                    if np.linalg.norm(model.geo2NED(entity, lat, lon, alt)) < r:
                        dead.add(ID)
                        break
        return dead

    def strike(self):  # 检查是否碰撞
        grid = spatialgrid(5)
        pos = {}  # 实体绝对坐标
        for ID, (entity, _) in self.entity.items():
            pos[ID] = -model.geo2NED(entity, *self.center)
            grid.add(ID, pos[ID])
        near = {}  # 邻近点缓存
        dis = defaultdict(dict)
        dead = set()
        for ID, (entity, _) in self.entity.items():
            key = grid.hash(pos[ID])
            if not key in near:
                near[key] = grid.getnear(pos[ID])
            for x in near[key]:
                if ID != x:
                    if not x in dis[ID]:
                        dis[ID][x] = dis[x][ID] = np.linalg.norm(pos[x] - pos[ID])
                    if dis[ID][x] < grid.size:
                        dead.add(x)
                        dead.add(ID)
                        if type(entity) == aim120c:  # 可能误截获
                            entity.TargetID = x
        return dead, pos

    def main(self, time=30, exist_ok=False):  # 仿真主函数
        """
        time:float,仿真最大分钟
        exist_ok:bool,是否覆盖最近acmi文件
        """
        queue = [mu.Queue() for _ in range(3)]
        mu.Process(target=self.redconsumer, args=queue[::2], daemon=True).start()
        mu.Process(target=self.blueconsumer, args=queue[1:], daemon=True).start()
        log = tacview(exist_ok)
        log.logtime(0)
        exist = {"Red": len(self.red), "Blue": len(self.blue)}
        self.entity = {}
        for entity in self.red:
            entity = f16(**entity)
            log.loginit(entity, "Red")
            self.entity[entity.ID] = entity, "Red"
        for entity in self.blue:
            entity = f16(**entity)
            entity.radarR *= 4 / 3  # 蓝方雷达距离优势
            log.loginit(entity, "Blue")
            self.entity[entity.ID] = entity, "Blue"
        dt = next(iter(self.entity.values()))[0].fdm.get_delta_t()  # 仿真步长
        # 主循环
        for step_num, time in enumerate(np.arange(0, 60 * time, dt)):
            for ID in self.restrict():
                entity, color = self.entity.pop(ID)
                if color in exist:
                    exist[color] -= 1
                else:
                    entity.TargetID = None  # 导弹失效
                log.logdestroy(ID)
            dead, pos = self.strike()
            for ID in dead:
                entity, color = self.entity.pop(ID)
                if color in exist:
                    exist[color] -= 1
                else:
                    entity.state = 2  # 导弹击中
                log.logdestroy(ID)
            if not all(exist.values()):
                break
            log.logtime(time)
            for ID1, (entity, _) in self.entity.items():
                if type(entity) == f16:
                    isNTS = {x.EnemyID: x.isNTS for x in entity.FoundEnemyList}
                    entity.FoundEnemyList.clear()
                    entity.AlarmList.clear()
                    for ID2, (target, _) in self.entity.items():
                        if ID1 == ID2:
                            continue
                        info, rel, std = model.find(entity, target, pos[ID2] - pos[ID1])
                        if info.TargetDis < 5e3:  # 告警
                            entity.alarm(ID2, info.TargetYaw, type(target).__name__)
                        # 雷达发现敌机
                        if type(target) == f16 and inradar(entity, info):
                            entity.find(info, rel, std, isNTS.get(ID2, False))
                elif entity.TargetID in self.entity:
                    ID2 = entity.TargetID
                    target = self.entity[ID2][0]
                    info, rel, std = model.find(entity, target, pos[ID2] - pos[ID1])
                    if inradar(entity, info):
                        aim120c.find(entity, info, rel, std)
            for entity, color in self.entity.values():  # 无人机信息发送
                if type(entity) == f16:
                    i = ["Red", "Blue"].index(color)
                    queue[i].put((entity.getinfo(), step_num))
            mislist = []  # 新发射导弹列表
            for entity, _ in self.entity.values():
                if type(entity) == aim120c:
                    old = entity.state
                    entity.step()
                    if not old and entity.state:
                        log.loginit(entity, "White", "AIM-120C")  # 主动雷达开启
                else:
                    ID, cmd = queue[2].get()
                    f16.step(self.entity[ID][0], cmd)
                    for enemy in self.entity[ID][0].FoundEnemyList:
                        if enemy.isNTS:
                            log.logNTS(ID, enemy.EnemyID)  # 持续显示锁定
                    if 1 == cmd.engage and MissileTrackList[ID]:
                        M = MissileTrackList[ID][-1]
                        if not M.ID in self.entity and M.TargetID and M.state < 2:
                            mislist.append(M)
                            log.loginit(M, "White", "AIM-120C", 0)  # 新发射导弹
                log.logstep(entity)
            for M in mislist:
                self.entity[M.ID] = M, "White"
        for q in queue[:-1]:
            q.put(None)
        print("仿真结束\n无人机剩余:", exist)
