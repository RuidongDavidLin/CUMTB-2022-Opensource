#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import json
import rospy
import actionlib
import math
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import std_srvs.srv
from std_msgs.msg import Bool, Int16
from math import atan2, sin, cos, sqrt
from threading import Lock, Event, Thread
from copy import deepcopy
import tf.transformations

# 导航目标发送类
class ROSNavNode(object):
    def __init__(self, dest_D=1):
        rospy.init_node("main_node", anonymous=False)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.odom_subsciber = rospy.Subscriber("/odom", Odometry, self._get_info)
        self.odom_pose_x = 0.0
        self.odom_pose_y = 0.0
        self._info_lock = Lock()

        # 一定要有这一行，读 actionlib 源码可以看到 client 和 server 会在建立连接时进行协商，然后丢掉第一个 goal
        # http://docs.ros.org/en/jade/api/actionlib/html/action__client_8py_source.html
        self.client.wait_for_server()
        self.pass_thres_radius = 0.05
        self.dest_D = dest_D

    def to_euler_angles(self, w, x, y, z):
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return y * 180 / math.pi

    def _get_info(self, msg):
        """
        获取base_driver发来的odometry消息
        """

    def gate_small_y(self, dis_y):
        """
        判断车辆的位置是否过线（数值小于Y轴）
        """

    def gate_big_y(self, dis_y):
        """
        判断车辆的位置是否过线（数值大于Y轴）
        """

    def gate_small_x(self, dis_x):
        """
        判断车辆的位置是否过线（数值小于X轴）
        """

    def gate_big_x(self, dis_big_x):
        """
        判断车辆的位置是否过线（数值大于X轴）
        """

    def gate_distance(self, dis_x, dis_y, radius=0.3):
        """
        计算并判断车辆当前的位置与设定位置的欧氏距离
        同时每个不同的出发点的对应冲刺点坐标有所不同，详见Readme.md文档
        """


    def get_pose_xy(self):
        "获取并返回当前坐标 X，Y"


    def check_if_pose_near(self, x, y, radius=0.8):
        """
        检测是否接近目标点，
        如果接近返回True，反之False
        radius 默认值为0.8m
        """


    def get_pose(self, id):
        """
        从文件中读取中间点位置,
        点位置的命名格式为Pose_id,
        只需要输入字符串id即可,
        eg. id = 'A1'
        """
        with open("/home/ucar/ucar_ws/src/ucar_nav/scripts/pose/pose_{}.json".format(id), "r") as f:
            text = json.loads(f.read())
            self.pos_x = text["position"]["x"]
            self.pos_y = text["position"]["y"]
            self.pos_z = text["position"]["z"]
            self.ori_x = text["orientation"]["x"]
            self.ori_y = text["orientation"]["y"]
            self.ori_z = text["orientation"]["z"]
            self.ori_w = text["orientation"]["w"]

    def get_pose_Dest(self):
        """
        从文件中读取终点位置
        若无参数则默认get_poseD1
        """
        with open("/home/ucar/ucar_ws/src/ucar_nav/scripts/pose/pose_Dest" + str(self.dest_D) + ".json", "r") as f:
            text = json.loads(f.read())
            self.pos_x = text["position"]["x"]
            self.pos_y = text["position"]["y"]
            self.pos_z = text["position"]["z"]
            self.ori_x = text["orientation"]["x"]
            self.ori_y = text["orientation"]["y"]
            self.ori_z = text["orientation"]["z"]
            self.ori_w = text["orientation"]["w"]

    def goal_pose(self):
        """
        构造 goal
        """
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = self.pos_x
        self.goal.target_pose.pose.position.y = self.pos_y
        self.goal.target_pose.pose.position.z = self.pos_z
        self.goal.target_pose.pose.orientation.x = self.ori_x
        self.goal.target_pose.pose.orientation.y = self.ori_y
        self.goal.target_pose.pose.orientation.z = self.ori_z
        self.goal.target_pose.pose.orientation.w = self.ori_w

    def send_goal(self):
        """
        发送 goal
        """
        self.goal_pose()
        self.client.send_goal(self.goal)

    def send_goal_with_id(self, id):
        pass

    def get_state(self):
        """
        判断目标完成情况，若完成目标则返回 True
        """

    def wait_for_goal_reached(self):
        pass

    def get_topic(self):
        """
        获取 Topic list，返回参数为 list
        好像没用到！离谱！
        """
        return rospy.get_published_topics()


# 语音节点通讯类
class ROSYuyinNode(object):
    def __init__(self):
        self.start_lable_subscriber = rospy.Subscriber("/start_lable", Bool, self._get_start)
        self.get_start = False

    def _get_start(self, lable):
        """
        唤醒成功
        """
        self.get_start = lable

# 车辆控制运动类
class SimplePosePublisher:
    def __init__(self, freq=18):
        pass

    def _pub_thread_entry(self):
        """终止程序时通知event，防止卡死"""

    def _odom_cb(self, odom):
        pass

    def get_odom(self):
        pass

    @staticmethod
    def _odom_to_pose(odom):
        pass

    def get_pose(self):
        pass

    @staticmethod
    def _wrapAngle(x):
        pass

    @staticmethod
    def _xyz_to_twist(x, y, yaw):
        pass

    def _calc_vel(self, odom):
        pass

    @staticmethod
    def _xyz_to_odom(x, y, yaw):
        pass

    def send_goal(self, goal, tolerance, proportion, max_vel):
        pass

    def wait(self):
        pass

    def send_goal_and_wait(self, *args, **kwargs):
        self.send_goal(*args, **kwargs)
        self.wait()


def main():
    n = ROSNavNode()

    print("send goal")
    n.send_goal()
    n.get_state()

    time.sleep(20)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n")
        print("操作已取消")
        exit(0)
