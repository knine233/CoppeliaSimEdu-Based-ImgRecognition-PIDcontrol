import time

import matplotlib.pyplot as plt
import numpy as np

"""
小车朝向表示
[7    0    1]
[6   car   2]
[5    4    3]
"""
# opencv中图像坐标用（y,x）访问且y轴方向向下，但我们下面调转xy因此这还是(x,y)
_Orients2coor = [(0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1)]
_Can_reach_ori = [[7, 0, 1], [0, 1, 2], [1, 2, 3], [2, 3, 4],
                 [3, 4, 5], [4, 5, 6], [5, 6, 7], [6, 7, 0]]

class A_star():
    def __init__(self, reporter, obs_dist, begin_ori, oriLimit=True, h_method="ou"):
        self.st, self.goal = reporter._start_position, reporter._goal_position
        self.obs_dist = obs_dist
        self.obstacle_map = reporter._obstacle_mask     # obstacle_map=0表示有障碍物
        self.size = self.obstacle_map.shape
        self.oriMap = np.ones(self.size).astype('int') * -1           # 记录小车到达每个点的朝向，初始为-1
        self.oriMap[self.st[0], self.st[1]] = begin_ori
        self.g = np.ones(self.size) * -1                # 用g是否=-1来表示是否未进入过openset
        self.f = np.ones(self.size) * np.inf         # 用f=1000000表示进入了closet
        # 用最小堆管理openset，注意两个set均为ndarray
        self.openset = []
        self.last = np.zeros((self.size[0], self.size[1], 2))
        self.obstacle_map = self.map_process(self.obstacle_map)
        self.oriLimit = oriLimit
        self.h_methods = h_method
    # 对给定的点用距离30的障碍线包围
    def obs_enlarge(self, map, x, y, dist):
        size = map.shape[0]
        if 0 <= y - 30 < size:
            for i in range(-dist, dist + 1):
                newx = x + i
                if 0 <= newx < size:
                    map[newx, y - 30] = 2
        if 0 <= y + 30 < size:
            for i in range(-dist, dist + 1):
                newx = x + i
                if 0 <= newx < size:
                    map[newx, y + 30] = 2
        if 0 <= x - 30 < size:
            for i in range(-dist, dist + 1):
                newy = y + i
                if 0 <= newy < size:
                    map[x - 30, newy] = 2
        if 0 <= x + 30 < size:
            for i in range(-dist, dist + 1):
                newy = y + i
                if 0 <= newy < size:
                    map[x + 30, newy] = 2
        return

    # 扩大障碍物，更新地图
    def map_process(self, map):
        new_map = map.copy()
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i, j] == 0:
                    self.obs_enlarge(new_map, i, j, self.obs_dist)
        new_map = np.where(new_map == 1, 1, 0).astype('int')
        return new_map

    def cal_h(self, cur):
        """
        计算启发式，有三种模式：欧几里得距离、曼哈顿距离、DIjkstra方法
        """
        if self.h_methods == "ou":
            return np.sqrt((cur[0]-self.goal[0])**2 + (cur[1]-self.goal[1])**2)
        elif self.h_methods == "M":
            return abs(cur[0]-self.goal[0]) + abs(cur[1]-self.goal[1])
        elif self.h_methods == "Dijkstra":
            return 0
        else:
            raise AssertionError("Expect h_method is one of below choices:\n ou, M, Dijkstra")


    def check_neighbors(self, cur):
        x, y = cur
        curg = self.g[x,y]
        curOri = self.oriMap[x,y]
        if self.oriLimit:
            orientations = _Can_reach_ori[curOri]
        else:
            orientations = range(8)
        for orientation in orientations:
            i, j = _Orients2coor[orientation]
            if orientation in [0, 2, 4, 6]:
                step_length = 1
            else:
                step_length = 1.4
            newx = x + i
            newy = y + j
            if not (0 <= newx < self.size[0] and 0 <= newy < self.size[1]):
                continue
            # 节点可通过且不在关闭列表
            if self.f[newx,newy] != 1000000 and self.obstacle_map[newx][newy] == 1:
                # 不在openset
                if self.g[newx,newy] == -1:
                    self.g[newx, newy] = curg + step_length
                    # self.openset = np.concatenate(self.openset,np.asarray(newx, newy, self.cal_h(cur)))
                    self.f[newx, newy] = curg + step_length + self.cal_h([newx,newy])
                    self.last[newx, newy] = [cur[0],cur[1]]
                    self.oriMap[newx, newy] = orientation
                elif self.f[newx, newy] > curg + step_length + self.cal_h([newx,newy]):
                    self.g[newx, newy] = curg + step_length
                    self.f[newx, newy] = curg + step_length + self.cal_h([newx, newy])
                    self.last[newx, newy] = [cur[0], cur[1]]
                    self.oriMap[newx, newy] = orientation
        return

    def search(self):
        cur = self.st
        self.g[cur[0],cur[1]] = 0
        self.f[cur[0],cur[1]] = self.cal_h(cur)
        self.openset.append(cur)
        count = 0
        time1 = time.time()
        while (cur != self.goal).any() and len(self.openset)>0:
            # 遍历周边点, 获取新的cur
            self.check_neighbors(cur)
            # 将该节点加入closeset，
            self.f[cur[0],cur[1]] = 1000000
            # 寻找下一个cur(最小f的点)
            idx = self.f.argmin()
            # 无解
            if self.f[int(idx/self.size[0]), int(idx%self.size[0])] == 1000000:
                return None
            cur = [int(idx/self.size[0]), int(idx%self.size[0])]
            count += 1
            # if count % 10000 == 0:
            #     G = self.g
            #     plt.figure()
            #     plt.imshow(G)
            #     plt.show()
        # 跳出循环
        if not (cur == self.goal).all():
            print("no way!")
            return None
        time2 = time.time()
        print(f"路径规划耗时:{time2-time1}")
        # 看看已遍历点
        G = self.g
        plt.figure()
        plt.imshow(G)
        plt.colorbar()
        plt.show()
        # 获取答案
        path = self.getPath(cur)
        return path

    def getPath(self, cur):
        """
        返回点cur到起点的最短路径，格式为2*n的ndarray
        """
        path = np.array([cur[0], cur[1]]).reshape(1,-1)
        parent = self.last[cur[0], cur[1]]
        while not (cur == self.st).all():
            new_way = np.asarray((parent[0],parent[1])).reshape(1,-1)
            path = np.concatenate((new_way,path), axis=0)
            cur = parent.astype('int')
            # t1,t2 = cur[0], cur[1]
            # t1 = self.last[cur[0], cur[1]]
            parent = self.last[cur[0], cur[1]]
        return path


class Dijkstra():
    def __init__(self, reporter, obs_dist, begin_ori, oriLimit=True):
        self.st, self.goal = reporter._start_position, reporter._goal_position
        self.obs_dist = obs_dist
        self.obstacle_map = reporter._obstacle_mask  # obstacle_map=0表示有障碍物
        self.size = self.obstacle_map.shape
        self.oriMap = np.ones(self.size).astype('int') * -1  # 记录小车到达每个点的朝向，初始为-1
        self.oriMap[self.st[0], self.st[1]] = begin_ori
        self.g = np.ones(self.size) * -1  # 用g是否=-1来表示是否未进入过openset


class PI_Control():
    def __init__(self,kp,ki,speed,n=5,max_signal=5,min_signal=0):
        self.kp = kp
        self.ki = ki
        self.i_memory = [0] * n
        self.iter = 0
        self.base = speed
        self.maxsig = max_signal
        self.minsig = min_signal

    def cal_result(self, error, limit_max=None):
        # 比例部分
        if limit_max != None:
            if error > limit_max:
                proportion = self.base * self.kp
            else:
                proportion = self.base * error / limit_max * self.kp
        else:
            proportion = self.base * error * self.kp

        # 积分部分
        self.i_memory[self.iter] = error
        self.iter = (self.iter + 1) % len(self.i_memory)
        sum_err = sum(self.i_memory)
        i = self.base * sum_err * self.ki
        result = proportion + i
        result = self.suitTolimit(result)
        return result

    def suitTolimit(self, speed):
        # 获取正负号
        if speed > 0:
            i = 1
        else:
            i = -1
        speed = min(self.maxsig, abs(speed))
        speed = max(self.minsig, abs(speed))
        return speed * i