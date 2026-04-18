from .utils import *
from os.path import dirname


def expm(A):  # 9阶Pade近似求矩阵指数
    j = np.linalg.norm(A, 1)  # 1范数
    j = max(0, 2 + int(np.floor(np.log2(j)))) if 0 < j else 0
    A /= 2**j
    E = np.eye(len(A))
    N = D = 0
    k = 1, 2, 28 / 3, 84, 1680, 30240, 1209600, 3024e4, 3024e5, 108864e4
    for i, x in enumerate(k):
        N += E / x
        D += (-1) ** i * E / x
        E @= A
    E = np.linalg.solve(D, N)
    for _ in range(j):
        E @= E
    return E


def psd(P):  # 确保P半正定
    val, vec = np.linalg.eigh((P + P.T) / 2)
    return vec @ np.diag(np.maximum(1e-12, val)) @ vec.T


class Kalman:
    H = np.hstack((np.eye(6), np.zeros((6, 3))))  # 观测矩阵

    def seta(self, a):
        # a:float,机动频率(1/s)
        if self.a != a:  # Singer模型单轴连续状态矩阵
            A = np.array(((0, 1, 0), (0, 0, 1), (0, 0, -a)))
            Q = np.zeros((6, 6))
            Q[2, 4] = self.q
            Q[:3, :3] = -A
            Q[3:, 3:] = A.T
            A, Q = map(expm, (A * self.dt, Q * self.dt))
            Q = A @ Q[:3, 3:] @ A.T
            Q = (Q + Q.T) / 2
            self.a = a
            self.F = np.eye(9)  # 状态转移矩阵
            self.Q = np.zeros((9, 9))  # 过程噪声协方差矩阵
            for i in range(0, 7, 3):
                self.F[i : 3 + i, i : 3 + i] = A
                self.Q[i : 3 + i, i : 3 + i] = Q

    def __init__(self, dt, std, q=5):
        """
        dt:float,滤波周期(s)
        std:np.ndarray,距离,速度标准差
        q:float,连续过程噪声功率谱密度(m^2/s^5)
        """
        self.dt = dt
        self.q = q
        self.X = np.zeros(9)  # 目标NED系绝对坐标(m),速度(m/s),加速度(m/s^2)
        # 状态估计误差协方差矩阵
        self.P = np.diag(np.repeat(np.append(std, 20) ** 2, 3))
        self.a = None
        self.seta(0.1)

    def predict(self):  # 绝对状态预测,不依赖导弹运动
        # 状态预测
        self.X = self.F @ self.X
        # 协方差预测
        self.P = psd(self.F @ self.P @ self.F.T + self.Q)

    def update(self, pos, vNED, std):
        """
        pos:np.ndarray,目标绝对坐标观测
        vNED:np.ndarray,目标绝对速度观测
        """
        R = np.diag(np.repeat(std**2, 3))  # 测量噪声协方差矩阵
        # 卡尔曼增益
        K = np.linalg.solve((self.H @ self.P @ self.H.T + R).T, self.H @ self.P.T).T
        # 状态更新
        self.X += K @ (np.hstack((pos, vNED)) - self.H @ self.X)
        # Joseph形式协方差更新
        I_KH = np.eye(9) - K @ self.H
        self.P = psd(I_KH @ self.P @ I_KH.T + K @ R @ K.T)

    def get(self):
        return self.X[:3].copy(), self.X[3:6].copy(), self.X[6:].copy()


class aim120c(model):
    def __init__(self, drone: model, info: EnemyInfo):
        """
        drone:载机实例
        info:目标敌机信息
        """
        super().__init__(dirname(__file__))
        # self.fdm.set_debug_level(5)
        self.fdm.load_model_with_paths("aim120c", ".", ".", ".", False)
        self.state = 0  # 0:中制导,1:末制导,2:已击中
        self.radarR = 5e3
        self.radarAngle = np.deg2rad(20)
        self.TargetID = info.EnemyID
        self.geo = drone.NED2geo(
            drone.los2NED(info.TargetYaw, info.TargetPitch, info.TargetDis)
        )  # 发射时目标经纬高(原点)
        self.data = ()  # 目标最新绝对信息
        self.kalman = Kalman(self.fdm.get_delta_t(), sigma(info.TargetDis))
        self.kalman.X[3:6] = info.vNED  # 速度初始化
        sy, sp = map(np.sin, (drone.Yaw, drone.Pitch))
        cy, cp = map(np.cos, (drone.Yaw, drone.Pitch))
        # 前方10m,避免炸到载机
        lat, lon, alt = model.NED2geo(drone, 10 * np.array((cp * cy, cp * sy, -sp)))
        # 载机状态继承
        lat, lon, head, pitch = np.rad2deg((lat, lon, drone.Yaw, drone.Pitch))
        super().ready(lat, lon, alt, drone.Mach_M, head, pitch, 0, 113)

    def find(self, info: EnemyInfo, pos, std):
        # pos:np.ndarray,目标相对坐标向量
        self.data = pos - super().geo2NED(*self.geo), info.vNED, std

    def guide(self):
        pos, vel, aNED = self.kalman.get()
        # 相对坐标 = 目标绝对坐标 + 导弹 -> 原点
        pos += super().geo2NED(*self.geo)
        # 绝对速度转相对速度
        vel -= self.vNED
        dis = float(np.linalg.norm(pos)) or 1
        if not self.state and dis <= self.radarR:
            self.state = 1
        los = pos / dis  # 单位向量
        # 径向接近速度
        Vc = max(1e-3, np.dot(-vel, los))
        # 避免末端指令奇异
        tgo = max(0.5, dis / Vc)
        an = aNED - np.dot(aNED, los) * los  # 法向加速度
        N = 4  # 比例基准值
        if self.state == 1:  # APN末制导
            # 目标机动补偿,末端衰减
            comp = N / 2 * an * min(1, tgo / 2)
            acmd = N * Vc * np.cross(los, vel) / dis + comp
        else:  # 0控脱靶量中制导
            # 法向速度
            vn = vel - np.dot(vel, los) * los
            acmd = N * min(1, 0.8 + 0.02 * tgo) * (vn / tgo + an / 2)
        acmd[2] += 9.80665
        return acmd

    def step(self):
        self.kalman.seta(0.1 + (not self.data))  # 目标丢失,加速度衰减增大
        self.kalman.predict()
        if self.data:
            self.kalman.update(*self.data)
            self.data = ()
        if 1 < self.fuel or 50 < -self.vNED[2]:  # 加速爬升
            return super().step2(0, 5)
        # 制导
        acmd = super().rotate().T @ self.guide()  # NED转体轴
        super().step2(np.arctan2(acmd[1], -acmd[2]), -acmd[2] / 9.80665)
