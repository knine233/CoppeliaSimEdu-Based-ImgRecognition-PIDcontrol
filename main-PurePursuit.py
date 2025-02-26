import time
from copy import deepcopy

import numpy as np
import matplotlib.pyplot as plt
import cv2
from reporter import Reporter
from tools import A_star, PI_Control

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py“')
    print ('--------------------------------------------------------------')

import math
import sys
import sim

color_dist = {'red': {'Lower': np.array([60, 0, 0]), 'Upper': np.array([255, 50, 50])},
              'blue': {'Lower': np.array([0, 0, 60]), 'Upper': np.array([50, 50, 255])},
              'green': {'Lower': np.array([0, 150, 0]), 'Upper': np.array([60, 255, 60])},
              'white': {'Lower':np.array([225, 225, 225]), 'Upper':np.array([245, 245, 245])},
              'black': {'Lower':np.array([0, 0, 0]), 'Upper':np.array([1, 1, 1])}
              }
h_method = "M"      # chose from:"M", "ou", "Dijkstra"

def smooth(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
    """
    Creates a smooth path for a n-dimensional series of coordinates.
    Arguments:
        path: List containing coordinates of a path
        weight_data: Float, how much weight to update the data (alpha)
        weight_smooth: Float, how much weight to smooth the coordinates (beta).
        tolerance: Float, how much change per iteration is necessary to keep iterating.
    Output:
        new: List containing smoothed coordinates.
    """
    new = deepcopy(path)
    dims = len(path[0])
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(new) - 1):
            for j in range(dims):

                x_i = path[i][j]
                y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                y_i_saved = y_i
                y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
                new[i][j] = y_i
                change += abs(y_i - y_i_saved)
    return new

def check_returnCode(rses, line_number):     # 检查return code，若return code不正常，停止程序
    for i in range(len(rses)):
        if not rses[i] == 0:
            print("return code error")
            print(rses)
            print(f"at line{line_number}")
            sys.exit()
    return

def pure_pursuit(cur_pos, cur_ori, going_path):
    # 找到最近路径点并计算前瞻点
    # 默认已经cut path了
    lookahead_point = going_path[0]
    # 计算路径偏差向量
    path_error_vector = lookahead_point - cur_pos
    path_error_angle = np.arctan2(path_error_vector[0], path_error_vector[1])
    # 计算期望的航向角
    desired_heading = cur_ori + np.sign(path_error_angle) * np.minimum(np.abs(path_error_angle), np.pi / 2)
    if desired_heading > np.pi:
        desired_heading -= 2*np.pi
    elif desired_heading < -np.pi:
        desired_heading += 2*np.pi
    return desired_heading

def calculate_wheel_speeds(desired_heading, current_heading, desired_speed):
    # 计算左右轮的角速度差
    delta_theta = desired_heading - current_heading
    left_wheel_speed = desired_speed
    right_wheel_speed = desired_speed
    # 这里我们简化处理，假设delta_theta很小，直接按比例分配速度差
    speed_difference = desired_speed * np.tan(delta_theta)
    right_wheel_speed -= speed_difference
    left_wheel_speed += speed_difference
    return left_wheel_speed, right_wheel_speed

# 作用是删除路径中已经过以及附近的路径点
def cut_path(path, x, y, scale=20):
    last_iter = 0
    for i in range(len(path)):
        yp, xp = path[i]
        dist = np.sqrt((x-xp)**2 + (y-yp)**2)
        if dist <= scale:
            last_iter = i + 1
            continue
        else:
            break
    return path[last_iter:]



print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1', -3,True,
                       True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print (f'Connected to remote API server,clientID = {clientID}')
    # enable the synchronous mode on the client:
    rc1 = sim.simxSynchronous(clientID,True)      #设置同步模式

    # start the simulation:
    rc2 = sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)      # 开始仿真
    check_returnCode([rc1], 106)
    # 第一次探测器的return code为8，所以先trigger一次，跳过第一次的错误
    sim.simxSynchronousTrigger(clientID)
    currentSimulationTime = -1
    # simxGetObjectHandle返回元组，第一个元素是return code，第二个才是所需
    rc1,LMotor = sim.simxGetObjectHandle(clientID,"bubbleRob_leftMotor",sim.simx_opmode_blocking)
    rc2,RMotor = sim.simxGetObjectHandle(clientID,"bubbleRob_rightMotor",sim.simx_opmode_blocking)
    check_returnCode([rc1,rc2], 113)

    # TODO:获取camera句柄并开启数据流
    rc1,camera = sim.simxGetObjectHandle(clientID,"top_view_camera",sim.simx_opmode_blocking)
    check_returnCode([rc1], 117)
    rc1 = sim.simxGetVisionSensorImage(clientID,camera,0,sim.simx_opmode_streaming)
    if (rc1[0] == sim.simx_return_novalue_flag):
        print("camara ready!")

    sim.simxSynchronousTrigger(clientID)

    # TODO : Do what you want below
    straight_speed = math.pi        # 直行速度系数
    turning_speed = 1               # 转向速度系数

    # TODO:初始化Reporter类，保存初始地图
    repo = Reporter()
    results = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_buffer)
    check_returnCode([results[0]], 131)
    sight = np.array(results[2], dtype=np.int16) + 256
    sight = sight.astype('uint8').reshape((results[1][0], results[1][1], 3))
    sight = np.flipud(sight)
    np.save("map.npy", sight)
    repo.log_init_image(sight)

    # TODO: 规划起点与终点
    # 提取颜色(r、g、b)，获取他们的掩码图片，
    # 然后将sight转化为灰度图进行识别轮廓，获取disc轮廓信息
    obstacle_s = cv2.inRange(sight, color_dist['white']['Lower'], color_dist['white']['Upper'])
    obstacle_disc = cv2.findContours(obstacle_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_s = cv2.inRange(sight, color_dist['red']['Lower'], color_dist['red']['Upper'])
    red_disc = cv2.findContours(red_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
    blue_s = cv2.inRange(sight, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
    blue_disc = cv2.findContours(blue_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
    green_s = cv2.inRange(sight, color_dist['green']['Lower'], color_dist['green']['Upper'])
    green_disc = cv2.findContours(green_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
    # 根据上面的轮廓信息获取各个圆盘的位置信息
    # 保存为障碍物掩码图像,np.where函数接受三个参数：一个条件（arr == 255），一个当条件为真时的值（0），以及一个当条件为假时的值（1）
    # 障碍物块为0
    obs_swapped = np.where(obstacle_s == 255, 0, 1)
    repo.log_obstacle_mask(obs_swapped)

    gx, gy = green_disc.reshape(-1, 2).mean(axis=0, dtype='int16')
    rx, ry = red_disc.reshape(-1, 2).mean(axis=0, dtype='int16')
    bx, by = blue_disc.reshape(-1, 2).mean(axis=0, dtype='int16')
    cur_x = (rx + bx) / 2
    cur_y = (ry + by) / 2
    start = np.asarray([cur_y, cur_x]).astype('int')
    goal = np.asarray([gy, gx]).astype('int')
    repo.log_start_position(start)
    repo.log_goal_position(goal)

    # TODO: 规划路径plan_path
    print("calculating path, please wait!")
    astar = A_star(repo, 30, 6, h_method=h_method)
    path = astar.search()
    # 路径平滑
    path = smooth(path, 0.01, 0.99)
    np.save("path.npy", path)
    # path = np.load('path.npy')
    path = path[::2]
    repo.log_plan_path(path)
    # repo.report_plan()

    # 隔两个点取一个点，原本的路径太密集了。
    going_path = path.copy()
    last_y, last_x = start
    base_speed = 16
    look_len = 20
    # TODO:Now step a few times:
    while currentSimulationTime < 60000:
        # 每个step，time+=50(0.05s)
        time1 = time.time()
        currentSimulationTime = sim.simxGetLastCmdTime(clientID)
        print("ct = ",currentSimulationTime)

        # get camera sight
        results = sim.simxGetVisionSensorImage(clientID,camera,0,sim.simx_opmode_buffer)
        check_returnCode([results[0]], 192)
        # result[2]是512*512*3的一维list
        sight = np.array(results[2], dtype=np.int16)+256
        sight = sight.astype('uint8').reshape((results[1][0],results[1][1],3))
        sight = np.flipud(sight)
        # 展示图片
        plt.cla()
        plt.imshow(sight)

        # 提取颜色(r、g、b)，获取他们的掩码图片，
        # 然后将sight转化为灰度图进行识别轮廓，获取disc轮廓信息
        red_s = cv2.inRange(sight, color_dist['red']['Lower'], color_dist['red']['Upper'])
        red_disc = cv2.findContours(red_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
        blue_s = cv2.inRange(sight, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
        blue_disc = cv2.findContours(blue_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]

        # 根据上面的轮廓信息获取各个圆盘的位置信息
        # 默认绿点不变
        rx, ry = red_disc.reshape(-1, 2).mean(axis=0, dtype='int16')
        bx, by = blue_disc.reshape(-1, 2).mean(axis=0, dtype='int16')
        cur_x = (rx + bx) / 2
        cur_y = (ry + by) / 2
        cur_pos = [cur_y, cur_x]
        going_path = cut_path(going_path, cur_x, cur_y, look_len)
        if len(going_path) != 0:
            tar_y, tar_x = going_path[0]
        else:
            tar_y, tar_x = goal
        car_ori = np.arctan2(ry - by, rx - bx)
        tar_ori = np.arctan2(tar_y - cur_y, tar_x - cur_x)
        print(f"目前小车位置({cur_x},{cur_y}),角度为{car_ori}rad")
        print(f"目前目标位置({tar_x},{tar_y}),在小车{tar_ori}rad的方位")

        left_speed, right_speed = calculate_wheel_speeds(tar_ori, car_ori, base_speed)

        # 判断是否抵达终点
        goal_error = np.sqrt((goal[1] - cur_x) ** 2 + (goal[0] - cur_y) ** 2)
        if goal_error < 3:
            print(f"足够接近目标")
            rc1 = sim.simxSetJointTargetVelocity(clientID, LMotor, 0, sim.simx_opmode_oneshot)
            rc2 = sim.simxSetJointTargetVelocity(clientID, RMotor, 0, sim.simx_opmode_oneshot)
            # check_returnCode([rc1, rc2])
            cur_pos = np.asarray([gy, gx])
            car_ori = float(car_ori)
            repo.log_robot_sim_state(cur_pos, car_ori, currentSimulationTime / 1000 + 0.05)
            repo.report_all()
            raise IndexError("结束了")
            # print("按任意键继续...")
            # input()  # 等待用户按下回车键
            # print("程序已恢复!")
        else:
            print(f"输出左轮速度为{left_speed}rad/s，右轮速度为{right_speed}rad/s")
            rc1 = sim.simxSetJointTargetVelocity(clientID, LMotor, left_speed, sim.simx_opmode_oneshot)
            rc2 = sim.simxSetJointTargetVelocity(clientID, RMotor, right_speed, sim.simx_opmode_oneshot)
            # check_returnCode([rc1, rc2])

        # 调用repo记录当前位置与方向
        # unsample_gap_count += 1
        # if unsample_gap_count % unsample_gap != 0:
        last_x = (last_x + cur_x) / 2
        last_y = (last_y + cur_y) / 2
        cur_pos = np.asarray([last_y, last_x])
        car_ori = float(car_ori)
        repo.log_robot_sim_state(cur_pos, car_ori, currentSimulationTime/1000)

        # getpingtime确保在当前仿真结束前，保持阻塞
        rc1 = sim.simxGetPingTime(clientID)
        rc2 = sim.simxSynchronousTrigger(clientID)
        check_returnCode([rc1[0],rc2], 270)
        print(f"loop cost time:{time.time()-time1}")
        print("-----------------end loop--------------------------------")

    # 停止仿真
    rc2 = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_discontinue)
    rc3 = sim.simxGetPingTime(clientID)
    check_returnCode([rc2,rc3], 277)

    # 断开链接
    rc1 = sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot)
    rc2 = sim.simxFinish(clientID)
    check_returnCode([rc1, rc2], 282)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')