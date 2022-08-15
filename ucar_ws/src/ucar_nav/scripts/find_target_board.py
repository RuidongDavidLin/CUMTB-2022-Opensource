#!/usr/bin/env python
# coding=utf-8
from __future__ import print_function

import rospy, math
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf.transformations import euler_from_quaternion
from sklearn.cluster import DBSCAN
from threading import Event, Lock


class LaserProjection:
    def __init__(self):
        pass

    def projectLaser(self, scan_in, range_cutoff=-1.0):
        pass


class TargetBoardFinder:
    def __init__(
        self,
        target_frame="map",
        scan_topic="/scan",
        tf_buffer=None,
        edge_dist_threshold=0.08,
        cluster_dist_threshold=0.2,
        cluster_min_samples=10,
        visualize=False,
    ):
        """
        激光雷达查找障碍物

        Args:
            target_frame (str, optional): 目标 tf 坐标系. Defaults to "map".
            scan_topic (str, optional): scan 话题的名字. Defaults to "/scan".
            tf_buffer (_type_, optional): 
            roi (tuple, optional): 
            edge_dist_threshold (float, optional):
            cluster_dist_threshold (float, optional):
            cluster_min_samples (int, optional):
            visualize (bool, optional):
        """
        pass

    def _scan_cb(self, scan):
        pass

    def _get_scan(self):
        pass

    def _wait_scan(self):
        pass

    def _check_tf_is_none(self):
        pass

    def register_tf(self):
        "在类内注册tf变换, 而不使用外面传进来的'tf_buffer'"
        pass

    def wait_for_scan_tf(self):
        "等待tf变换有效"
        pass

    def _get_scan_tf(self):
        "获取激光雷达消息和tf变换"
        pass

    def _project(self, scan, trans):
        "将激光雷达的数据投影到'target_frame'坐标系下"
        pass

    @staticmethod
    def bound_roi(points, roi):
        pass

    def _find_points_in_roi(self, roi):
        pass

    def find_points_in_roi(self, roi):
        pass

    def _limit_convex_hull(self, points):
        "删除距离凸包太近的点"
        pass

    def _cluster_points(self, points):
        "聚类"
        pass

    def find_target_board(self, roi, additional_points=None):
        """寻找目标板

        Returns:
            list[list[float, float, float, float, float]]: 一个列表, 保存每个板子最小外接矩形的[x, y, w, h, theta]
        """
        pass

    def show(self, blocking_time=-1):
        """显示图像

        Args:
            blocking_time (float): 显示时阻塞的时间, 单位秒, 小于0则永久阻塞
        """
        pass


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    rospy.init_node("find_target_board", anonymous=True)
    target_board_finder = TargetBoardFinder(
        target_frame="map",
        scan_topic="/scan",
        edge_dist_threshold=0.07,
        cluster_dist_threshold=0.20,
        cluster_min_samples=8,
        visualize=True,
    )
    target_board_finder.register_tf()
    target_board_finder.wait_for_scan_tf()
    boards = target_board_finder.find_target_board(
        roi=[
            # [6.0, -2.00],
            # [3.5, -2.0],
            # [3.5, -5.70],
            # [6.0, -5.70],
            # [3.96, -3.02],
            # [2.07, -2.99],
            # [2.15, -5.67],
            # [3.9, -5.73]
            [3.83, -3.02], [2.07, -2.99], [2.15, -5.67], [3.9, -5.73]
        ],
        additional_points=[
        [3.75, -3.03], [3.71, -5.47], [2.30, -5.45],[2.20,-3.0],[3.40,-2.94]
        ]
    )
    print(boards)

    # points = target_board_finder.find_points_in_roi(roi=[
    #     [5.68, -1.95],
    #     [3.66, -1.9],
    #     [3.73, -5.66],
    #     [6.0, -5.8],
    # ])
    # print(points.shape[0])
    target_board_finder.show()

    # while not rospy.is_shutdown():
    #     boards = target_board_finder.find_target_board()
    #     print(boards)
    #     target_board_finder.show(0.5)
