#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import time

from numpy import True_
import rospy
from ros_module_X import ROSNavNode, ROSYuyinNode, SimplePosePublisher
from interactive import show, get_bool_ans, get_str_ans, save
import socket
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from math import sin, cos
from threading import Event, Thread
import find_target_board
import math


# ----------------------------Function Labels-------------------------------#
# 用于存储部分功能的开启标签
# --------------------------------------------------------------------------#
fix_local_planner = False  # 是否启用局部固定路径规划
Pose_1 = True
Pose_2 = False  # 出于障碍物摆放的原因，故Pose_3设定为需要停止的点，不做标签化处理
LaserScan_detect = True  # 是否启动激光雷达探测障碍物方位功能
GateB = True                # 是否在C房间门口停车拍照
GateC = True                # 是否在C房间门口停车拍照
GateD = True                # 是否在D房间门口停车拍照
SlowB = False               # 接近B房间中心时是否减速
SlowC = True                # 接近C房间中心时是否减速
SlowD = True                # 接近D房间中心时是否减速

if LaserScan_detect:
    # 加载预设的点位区域
    roi_GateB = [[4.0, -1.8], [4.0, -2.4], [4.6, -2.4], [4.6, -1.8]]
    roi_CenterB = [[4.37, -3.45], [4.37, -3.95], [4.87, -3.95], [4.87, -3.45]]
    roi_find_board_B = [[6.0, -2.00], [3.5, -2.0], [3.5, -5.70], [6.0, -5.70]]
    additional_points_B = [[3.75,-2.58],[3.77, -5.50], [5.7, -5.52], [5.7, -2.03]]

    roi_GateC = [[2.98, -3.02], [2.56, -3.02], [2.55, -3.32], [2.94, -3.32]]
    roi_CenterC = [[3.38, -3.8], [2.65, -3.8], [2.7, -4.59], [3.28, -4.59]]
    roi_find_board_C = [[3.83, -3.02], [2.07, -2.99], [2.15, -5.67], [3.9, -5.73]]
    additional_points_C = [[3.75, -3.03], [3.71, -5.47], [2.30, -5.45],[2.20,-3.0],[3.40,-2.94]]

    roi_GateD = [[1.5, -1.95], [1.5, -2.45], [2.0, -2.45], [2.0, -1.95]]
    roi_CenterD = [[0.65, -3.35], [0.65, -3.85], [1.15, -3.85], [1.15, -3.35]]
    roi_find_board_D = [[2.33, -1.9], [-0.313, -1.9], [-0.37, -5.71], [2.40, -5.71]]
    additional_points_D = [[-0.213, -2.12], [-0.213, -5.54], [2.19, -5.52], [2.15, -2.06]]


    def merge_yaw(yaw_targets):
        # 把相邻的板子弄到一张图里
        merge_threshold = math.radians(30)
        if len(yaw_targets) < 2:
            return yaw_targets
        PI2 = math.pi * 2
        segments = []
        for i, yaw in enumerate(yaw_targets):
            if segments and yaw - yaw_targets[segments[-1][0]] <= merge_threshold:
                segments[-1][1] = i
            else:
                segments.append([i, i])
        if len(segments) > 1 and yaw_targets[segments[0][1]] + PI2 - yaw_targets[segments[-1][0]] <= merge_threshold:
            segments[0][0] = segments[-1][0]
            segments.pop()
        yaw_targets_merged = []
        for l, r in segments:
            if l <= r:
                cur = (yaw_targets[l] + yaw_targets[r]) / 2
            else:
                cur = (yaw_targets[l] + yaw_targets[r] + PI2) / 2
                if cur > math.pi:
                    cur -= PI2
            yaw_targets_merged.append(cur)
        return yaw_targets_merged
    
    # 实例化障碍物检测类
    LaserDetect = find_target_board.TargetBoardFinder(
        target_frame="map",
        scan_topic="/scan",
        edge_dist_threshold=0.07,
        cluster_dist_threshold=0.20,
        cluster_min_samples=8,
        visualize=False,
    )

def open_terminal(commands):
    """
    打开一个新的终端并执行命令
    未作参数检查，不要试图在传入的命令字符串中添加多余的引号，可能会引发错误
    ; exec bash 这个参数加上保证终端执行完指令后不会自动退出
    """
    cmd_list = []
    for cmd in commands:
        cmd_list.append(""" gnome-terminal --tab -e "bash -c '%s; exec bash' " >/dev/null  2>&1 """ % cmd)

    os.system(";".join(cmd_list))


def launch_pkg():
    """
    启动 roscore 与 语音唤醒模块
    """
    show("即将打开roscore以及语音模块")
    nav_cmd = [
        "roscore",
        "sleep 6; roslaunch xf_mic_asr_offline xf_mic_asr_offline.launch",
    ]
    open_terminal(nav_cmd)

    show("正在启动，请稍等...")
    time.sleep(5)


def main():

    # 启动roscore以及语音模块后链接视觉模组
    launch_pkg()
    rospy.init_node("main_node", anonymous=False)

    s = socket.socket()
    # 与摄像头模块建立socket通讯,视觉模组进入阻塞
    s.connect(("127.0.0.1", 39393))

    def snapshot_and_check(room):
        """
        room 为所处房间
        eg. room = b"A"
        """
        s.send(room)
        s.recv(1024)
        s.send(b"recog")
        return s.recv(1024) == b"finish"

    yuyinNode = ROSYuyinNode()
    # 等待语音节点成功启动
    time.sleep(6)

    # 启动导航模块
    nav_cmd1 = ["roslaunch ucar_nav ucar_navi.launch"]
    show("Pulling the navigation module up, please waiting...")
    open_terminal(nav_cmd1)

    # ---------------------------------------------------------------------------
    # 由于base_drvier.launch和ylidar.launch存在启动失败的情况，所以可能需手动重启
    # ---------------------------------------------------------------------------
    while not get_bool_ans("上述命令是否已全部在其他窗口正确执行？"):
        show('Close the 4th windows by press "Ctrl + c"! ')
        show("Starting navigation module again. Please check the 4th windows!")
        open_terminal(nav_cmd1)

    # 实例化启动导航辅助节点
    node = ROSNavNode()
    simple_pose_publisher = SimplePosePublisher()

    if LaserScan_detect:
        LaserDetect.register_tf()                       # 注册TF关系变换
        LaserDetect.wait_for_scan_tf()                  # 等待TF关系变换结果

    time.sleep(1)
    
    # 语音唤醒成功后直接解冻导航节点，小车开始移动
    show("请喊“小V小V")
    while not yuyinNode.get_start:
        time.sleep(0.01)

    # 开始导航任务
    show("开始执行导航任务")
    # 出发区出发时间
    T1 = time.time()

    if not fix_local_planner:
        # 前往目标点1（分段导航）
        if Pose_1:
            show("前往Pose_1")
            node.get_pose("1")
            node.send_goal()
            while not node.get_state():
                time.sleep(0.1)

        # 前往目标点2（分段导航）
        if Pose_2:
            show("前往Pose_2")
            node.get_pose("2")
            node.send_goal()
            # 判断车是否超过标定的X轴垂线
            node.gate_big_x(dis_big_x=3.0)

        # 前往目标点3 （分段导航）
        show("前往Pose_3")
        node.get_pose("3")
        node.send_goal()
        # 判断是否到达Pose_3
        while not node.get_state():
            time.sleep(0.1)
        # POSE_3点达到时间
        T2 = time.time()
        Dur2_1 = "%.5f" % (T2 - T1)
        show("出发点到 Pose_3 用时" + str(Dur2_1) + "秒")
    else:
        pass
        # 主要是太懒了，后面没写这一段
        # todo

    if LaserScan_detect:

        def roomB():
            if GateB :
                show("前往房间B门口位置")
                node.send_goal_with_id("B-Gate")

                # 当距离 B-Gate 超过0.5m时，检测是否有障碍物
                rate = rospy.Rate(5)
                while not node.check_if_pose_near(x=4.21, y=-2.14, radius=1.0):
                    test1 = time.time()
                    if LaserDetect.find_points_in_roi(roi=roi_GateB).shape[0] >= 2:
                        # 门口有板子，前往房间中央
                        break
                    rate.sleep()
                    print(time.time() - test1)
                else:  # 门口没板子
                    node.wait_for_goal_reached()
                    if snapshot_and_check(b"B"):
                        return  # 识别到目标，前往下一个房间
                    
            # 前往房间中央
            node.send_goal_with_id("B-Center")
            while not node.check_if_pose_near(4.62, -3.67, 1.0):
                if LaserDetect.find_points_in_roi(roi=roi_CenterB).shape[0] >= 2:
                    # 房间中央有板子
                    node.send_goal_with_id("B")
                    break
            else:
                if SlowB:
                    # 这里的减速停车方式很多样，方法不局限
                    pass
            node.wait_for_goal_reached()
            
            if snapshot_and_check(b"B"):
                return  # 识别到目标，前往下一个房间

            boards = LaserDetect.find_target_board(roi=roi_find_board_B, additional_points=additional_points_B)
            boards = LaserDetect.find_target_board(roi=roi_find_board_B, additional_points=additional_points_B)
            boards = LaserDetect.find_target_board(roi=roi_find_board_B, additional_points=additional_points_B)
            # 记录该次识别到的障碍物
            show(boards)
            cur_x, cur_y, _ = simple_pose_publisher.get_pose()
            yaw_targets = list(sorted(math.atan2(y - cur_y, x - cur_x) for x, y, _, _, _ in boards))
            yaw_targets = merge_yaw(yaw_targets)

            for yaw in yaw_targets:
                simple_pose_publisher.send_goal_and_wait(
                    (cur_x, cur_y, yaw), (1e6, math.radians(5)), (0.3, 3), (0.3, 3)
                )
                if snapshot_and_check(b"B"):
                    return  # 识别到目标，前往下一个房间

        roomB()
        T3 = time.time()
        Dur3_2 = "%.5f" % (T3 - T2)
        show("B房间完成识别，用时 " + Dur3_2 + " s")
        
    else:
        # 前往房间B，即Pose_B
        show("前往房间B-Pose_B!")
        node.get_pose("B")
        node.send_goal()
        while not node.get_state():  # 判断是否到达Pose_3
            time.sleep(0.1)
        T3 = time.time()  # POSE_B(房间B)点到达时间
        Dur3_2 = "%.5f" % (T3 - T2)
        show("到达到房间B—Pose_B!出发点到房间B用时" + str(Dur3_2) + "秒，开始采集数据")

        # 开始B房间采集
        state = b"unfinished"
        s.send(b"recog")  # 第一次发送采集指令)
        state = s.recv(1024)  # 第一次接收采集完成的信号
        if state != b"finish":
            node.get_pose("B1")  # 在B房间内移动后重新采集
            node.send_goal()
            while not node.get_state():
                time.sleep(0.1)
            show("到达该房间第二个识别点")
            s.send(b"recog")  # 发送再次进行识别的信号
            state = s.recv(1024)
            if state != b"finish":
                node.get_pose("B2")
                node.send_goal()
                time.sleep(0.05)
                node.send_goal()
                while not node.get_state():
                    time.sleep(0.1)
                show("到达该房间第三个识别点")
                s.send(b"recog")
                state = s.recv(1024)

    show("第一次采集结束")


    # 开始C房间采集
    if LaserScan_detect:

        def roomC():
            if GateC:
                show("前往房间C门口位置")
                node.send_goal_with_id("C-Gate")

                # 当距离 C-Gate 超过0.5m时，检测是否有障碍物
                while not node.check_if_pose_near(2.75, -3.3, 1.0):
                    if LaserDetect.find_points_in_roi(roi=roi_GateC).shape[0] >= 2:
                        # 门口有板子，前往房间中央
                        break
                else:
                    show("等待前往房间C门口")  # 门口没板子
                    node.wait_for_goal_reached()
                    if snapshot_and_check(b"C"):
                        return  # 识别到目标，前往下一个房间

            # 前往房间中央
            print("前往房间C中央")
            node.send_goal_with_id("C-Center")
            while not node.check_if_pose_near(3.0, -4.10, 1.0):
                if LaserDetect.find_points_in_roi(roi=roi_CenterC).shape[0] >= 2:
                    # 房间中央有板子
                    node.send_goal_with_id("C")
                    break
            else:
                if SlowC:
                    # 这里的减速停车方式很多样，方法不局限
                    pass
            node.wait_for_goal_reached()
            
            if snapshot_and_check(b"C"):
                return  # 识别到目标，前往下一个房间
            
            boards = LaserDetect.find_target_board(roi=roi_find_board_C, additional_points=additional_points_C)
            show(boards)

            cur_x, cur_y, _ = simple_pose_publisher.get_pose()
            yaw_targets = list(sorted(math.atan2(y - cur_y, x - cur_x) for x, y, _, _, _ in boards))
            yaw_targets = merge_yaw(yaw_targets)

            for yaw in yaw_targets:
                simple_pose_publisher.send_goal_and_wait(
                    (cur_x, cur_y, yaw), (1e6, math.radians(5)), (0.3, 3), (0.3, 3)
                )
                if snapshot_and_check(b"C"):
                    return  # 识别到目标，前往下一个房间

        roomC()
        T4 = time.time()
        Dur4_3 = "%.5f" % (T4 - T3)
        show("C房间完成识别，用时 " + Dur4_3 + " s")
    
    else:
        # 前往房间C，即Pose_4
        show("前往房间C-Pose_C!")
        node.get_pose("C")
        node.send_goal()
        while not node.get_state():  # 判断是否到达Pose_C
            time.sleep(0.1)
        state = b"unfinished"
        s.send(b"recog")  # 第一次发送采集指令
        state = s.recv(1024)  # 第一次接收采集完成的信号
        if state != b"finish":
            node.get_pose("C1")  # 在C房间内移动后重新采集
            node.send_goal()
            while not node.get_state():
                time.sleep(0.1)
            show("到达该房间第二个识别点")
            s.send(b"recog")  # 发送再次进行识别的信号
            state = s.recv(1024)
            if state != b"finish":
                node.get_pose("C2")
                node.send_goal()
                time.sleep(0.05)
                node.send_goal()
                while not node.get_state():
                    time.sleep(0.1)
                show("到达该房间第三个识别点")
                s.send(b"recog")
                state = s.recv(1024)


    # 开始D房间采集
    if LaserScan_detect:

        def roomD():
            if GateD:
                show("前往房间D门口位置")
                node.send_goal_with_id("D-Gate")

                # 当距离 D-Gate 超过0.5m时，检测是否有障碍物
                while not node.check_if_pose_near(1.75, -2.2, 1.0):
                    if LaserDetect.find_points_in_roi(roi=roi_GateD).shape[0] >= 2:
                        # 门口有板子，前往房间中央
                        break
                else:  # 门口没板子
                    node.wait_for_goal_reached()
                    if snapshot_and_check(b"D"):
                        return  # 识别到目标，前往下一个房间

            # 前往房间中央)
            node.send_goal_with_id("D-Center")
            while not node.check_if_pose_near(0.9, -3.60, 1.0):
                if LaserDetect.find_points_in_roi(roi=roi_CenterD).shape[0] >= 2:
                    # 房间中央有板子
                    node.send_goal_with_id("D")
                    break
            else:
                if SlowD:
                    # 这里的减速停车方式很多样，方法不局限
                    pass
            node.wait_for_goal_reached()
			
            if snapshot_and_check(b"D"):
                return  # 识别到目标，前往下一个房间
            boards = LaserDetect.find_target_board(roi=roi_find_board_D, additional_points=additional_points_D)
            show(boards)
            
            cur_x, cur_y, _ = simple_pose_publisher.get_pose()
            yaw_targets = list(sorted(math.atan2(y - cur_y, x - cur_x) for x, y, _, _, _ in boards))
            yaw_targets = merge_yaw(yaw_targets)

            for yaw in yaw_targets:
                simple_pose_publisher.send_goal_and_wait(
                    (cur_x, cur_y, yaw), (1e6, math.radians(5)), (0.3, 3), (0.3, 3)
                )
                if snapshot_and_check(b"D"):
                    return  # 识别到目标，前往下一个房间

        roomD()
        T5 = time.time()
        Dur5_4 = "%.5f" % (T5 - T4)  # Pose_D(房间D)点到达时间
        show("D房间完成识别，用时 " + Dur5_4 + " s")
    else:
        
        show("前往房间D-Pose_D!")
        node.get_pose("D")
        node.send_goal()  # send目标点两次是因为某些原因使得第一次可能会失败
        time.sleep(1)
        node.send_goal()
        while not node.get_state():  # 判断是否到达Pose_D
            time.sleep(0.1)
        state = b"unfinished"
        s.send(b"recog")  # 第一次发送采集指令
        state = s.recv(1024)  # 第一次接收采集完成的信号
        if state != b"finish":
            node.get_pose("D1")  # 在D房间内移动后重新采集
            node.send_goal()
            while not node.get_state():
                time.sleep(0.1)
            show("到达该房间第二个识别点")
            s.send(b"recog")  # 发送再次进行识别的信号
            state = s.recv(1024)
            if state != b"finish":
                node.get_pose("D2")
                node.send_goal()
                time.sleep(0.05)
                node.send_goal()
                while not node.get_state():
                    time.sleep(0.1)
                show("到达该房间第三个识别点")
                s.send(b"recog")
                state = s.recv(1024)
    
    show("前往终点！")
    node.get_pose(4)
    node.send_goal()
    time.sleep(0.1)
    node.send_goal()
    rate = rospy.Rate(5)
    while not node.check_if_pose_near(0.1,-0.8, 1.3):
        rate.sleep()
    else:
        node.get_pose_Dest()
        node.send_goal()
        show("Loading Teb_local_planner parameters ...")
        time.sleep(0.1)        
        node.send_goal()

        
    while not node.get_state():
        time.sleep(0.1)
    T6 = time.time()  # 到达任务领取区时间
    Dur_Total = "%.5f" % (T6 - T1)
    show("到达终点，任务完成，总共用时" + str(Dur_Total) + "秒")
    s.send(b"play")
    s.close()




if __name__ == "__main__":
    try:
        main()
        # 按ENTER键退出终端
        get_str_ans(" Press Enter")
    except KeyboardInterrupt:
        show("任务已取消")
        exit(0)
