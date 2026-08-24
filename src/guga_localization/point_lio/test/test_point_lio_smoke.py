#!/usr/bin/env python3

import math
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


@pytest.mark.launch_test
def generate_test_description():
    point_lio = launch_ros.actions.Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio_smoke",
        output="screen",
        parameters=[
            {
                "use_imu_as_input": False,
                "init_map_size": 10,
                "point_filter_num": 1,
                "space_down_sample": False,
                "common.lid_topic": "smoke/lidar",
                "common.imu_topic": "smoke/imu",
                "common.con_frame": False,
                "common.cut_frame": False,
                "prior_pcd.enable": False,
                "prior_pcd.prior_pcd_map_path": "",
                "prior_pcd.init_pose": [0.0, 0.0, 0.0],
                "preprocess.lidar_type": 1,
                "preprocess.scan_line": 4,
                "preprocess.blind": 0.1,
                "mapping.imu_en": False,
                "mapping.extrinsic_est_en": False,
                "mapping.gravity": [0.0, 0.0, -9.81],
                "mapping.gravity_init": [0.0, 0.0, -9.81],
                "mapping.extrinsic_T": [0.0, 0.0, 0.0],
                "mapping.extrinsic_R": [
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
                "publish.path_en": False,
                "publish.scan_publish_en": True,
                "publish.scan_bodyframe_pub_en": False,
                "publish.tf_send_en": False,
                "pcd_save.pcd_save_en": False,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [point_lio, launch_testing.actions.ReadyToTest()]
        ),
        {"point_lio": point_lio},
    )


class TestPointLioSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("point_lio_smoke_test")
        self.lidar_pub = self.node.create_publisher(
            CustomMsg, "smoke/lidar", 10
        )
        self.odom_received = threading.Event()
        self.cloud_received = threading.Event()
        self.last_odom = None
        self.node.create_subscription(
            Odometry, "aft_mapped_to_init", self._on_odom, 10
        )
        self.node.create_subscription(
            PointCloud2, "cloud_registered", self._on_cloud, 10
        )

    def tearDown(self):
        self.node.destroy_node()

    def _on_odom(self, msg):
        self.last_odom = msg
        self.odom_received.set()

    def _on_cloud(self, msg):
        if msg.width > 0:
            self.cloud_received.set()

    def _spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _make_lidar_frame(self, frame_index):
        msg = CustomMsg()
        stamp_ns = self.node.get_clock().now().nanoseconds + frame_index * 100_000_000
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = "livox_frame"

        for index in range(64):
            angle = 2.0 * math.pi * index / 64.0
            point = CustomPoint()
            point.offset_time = index * 100_000
            point.x = 3.0 * math.cos(angle)
            point.y = 3.0 * math.sin(angle)
            point.z = 0.02 * (index % 4)
            point.reflectivity = 100
            point.tag = 0x10
            point.line = index % 4
            msg.points.append(point)

        msg.point_num = len(msg.points)
        return msg

    def test_processes_lidar_and_publishes_outputs(self):
        self.assertTrue(
            self._spin_until(
                lambda: self.lidar_pub.get_subscription_count() > 0, 10.0
            ),
            "PointLIO did not subscribe to the smoke LiDAR topic",
        )

        for frame_index in range(3):
            self.lidar_pub.publish(self._make_lidar_frame(frame_index))
            rclpy.spin_once(self.node, timeout_sec=0.15)

        self.assertTrue(
            self._spin_until(self.odom_received.is_set, 10.0),
            "PointLIO did not publish odometry after receiving LiDAR data",
        )
        self.assertTrue(
            self._spin_until(self.cloud_received.is_set, 5.0),
            "PointLIO did not publish a registered point cloud",
        )
        pose = self.last_odom.pose.pose
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        self.assertTrue(all(math.isfinite(value) for value in values))
        self.assertEqual(self.last_odom.header.frame_id, "camera_init")
        self.assertEqual(self.last_odom.child_frame_id, "body")


@launch_testing.post_shutdown_test()
class TestPointLioShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, point_lio):
        launch_testing.asserts.assertExitCodes(proc_info, process=point_lio)
