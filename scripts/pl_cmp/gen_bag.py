#!/usr/bin/env python3
"""生成用于 Point-LIO 两个版本对比的合成 rosbag。

发布内容:
  /livox/lidar  livox_ros_driver2/msg/CustomMsg  10 Hz, 20000 点/帧
  /livox/imu    sensor_msgs/msg/Imu              200 Hz

场景: 8m x 8m x 3m 的封闭房间 (地面/天花板/四面墙)。
运动: 半径 2m 的圆周运动 (0.4 m/s, 0.2 rad/s) 叠加 1 Hz 垂直起伏, 姿态只有 yaw。
      点在其采样时刻的雷达系下表达, 因此天然带帧内运动畸变。

IMU 单位约定与 config/mid360.yaml 一致: acc_norm = 1.0 -> 加速度以 g 为单位,
角速度以 rad/s 为单位。
"""

import math
import os
import time

import numpy as np
import rclpy.serialization
import rosbag2_py
from builtin_interfaces.msg import Time
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

ROOM_HALF = 8.0
FLOOR_Z = 0.0
CEIL_Z = 3.0
RADIUS = 2.0
OMEGA = 0.2
HEIGHT = 0.5
BOB_A = 0.05
BOB_F = 1.0
G_M_S2 = 9.81

IMU_RATE = 200.0
LIDAR_RATE = 10.0
POINTS_PER_FRAME = 20000
ELEV_MIN = math.radians(-25.0)
ELEV_MAX = math.radians(30.0)
RANGE_SIGMA = 0.01
NO_RETURN_P = 0.01
T_START_IMU = 0.0
T_START_LIDAR = 1.0
T_END = 20.0

BIAS_GYRO = np.array([0.003, -0.002, 0.004])
BIAS_ACC = np.array([0.010, 0.010, 0.020])
SIGMA_GYRO = 0.002
SIGMA_ACC = 0.005

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
PL_DIR = os.environ.get("PL_CMP_DIR", os.path.join(WS, ".pl_cmp_bench"))

BAG_URI = os.path.join(PL_DIR, "bags", "mid360_synth")
SEED = 20260924


def position(t):
    return np.array([
        RADIUS * math.sin(OMEGA * t),
        RADIUS - RADIUS * math.cos(OMEGA * t),
        HEIGHT + BOB_A * math.sin(2.0 * math.pi * BOB_F * t),
    ])


def acceleration_world(t):
    return np.array([
        -RADIUS * OMEGA ** 2 * math.sin(OMEGA * t),
        RADIUS * OMEGA ** 2 * math.cos(OMEGA * t),
        -BOB_A * (2.0 * math.pi * BOB_F) ** 2 * math.sin(2.0 * math.pi * BOB_F * t),
    ])


def yaw(t):
    return OMEGA * t


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ray_plane_distance(origin, direction, axis, value):
    if abs(direction[axis]) < 1e-12:
        return None
    dist = (value - origin[axis]) / direction[axis]
    if dist <= 1e-6:
        return None
    return dist


def cast_ray(origin_world, direction_world):
    """在封闭房间内求最近命中距离。"""
    best = None
    for axis, value in ((0, ROOM_HALF), (0, -ROOM_HALF), (1, ROOM_HALF),
                        (1, -ROOM_HALF), (2, CEIL_Z), (2, FLOOR_Z)):
        dist = ray_plane_distance(origin_world, direction_world, axis, value)
        if dist is None:
            continue
        hit = origin_world + dist * direction_world
        if axis == 0 and (abs(hit[1]) > ROOM_HALF or not FLOOR_Z <= hit[2] <= CEIL_Z):
            continue
        if axis == 1 and (abs(hit[0]) > ROOM_HALF or not FLOOR_Z <= hit[2] <= CEIL_Z):
            continue
        if axis == 2 and (abs(hit[0]) > ROOM_HALF or abs(hit[1]) > ROOM_HALF):
            continue
        if best is None or dist < best:
            best = dist
    return best


def make_header(stamp_ns, frame_id):
    header = Header()
    header.stamp = Time(sec=int(stamp_ns // 1_000_000_000),
                        nanosec=int(stamp_ns % 1_000_000_000))
    header.frame_id = frame_id
    return header


def main():
    rng = np.random.default_rng(SEED)
    base_ns = int(time.time() * 1e9) + 5_000_000_000

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=BAG_URI, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        name="/livox/lidar", type="livox_ros_driver2/msg/CustomMsg",
        serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name="/livox/imu", type="sensor_msgs/msg/Imu",
        serialization_format="cdr"))

    imu_dt = 1.0 / IMU_RATE
    n_imu = int(round((T_END + 1.0 - T_START_IMU) / imu_dt)) + 1
    for k in range(n_imu):
        t = T_START_IMU + k * imu_dt
        stamp_ns = base_ns + int(round(t * 1e9))
        msg = Imu()
        msg.header = make_header(stamp_ns, "front_mid360")
        gyro = np.array([0.0, 0.0, OMEGA]) + BIAS_GYRO + rng.normal(0.0, SIGMA_GYRO, 3)
        specific_force = acceleration_world(t) - np.array([0.0, 0.0, -G_M_S2])
        acc = rot_z(yaw(t)).T @ specific_force / G_M_S2 + BIAS_ACC \
            + rng.normal(0.0, SIGMA_ACC, 3)
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])
        msg.orientation.w = 1.0
        writer.write("/livox/imu", rclpy.serialization.serialize_message(msg), stamp_ns)

    frame_dt = 1.0 / LIDAR_RATE
    n_frames = int(round((T_END - T_START_LIDAR) * LIDAR_RATE)) + 1
    total_points = 0
    for f in range(n_frames):
        t_frame = T_START_LIDAR + f * frame_dt
        stamp_ns = base_ns + int(round(t_frame * 1e9))
        msg = CustomMsg()
        msg.header = make_header(stamp_ns, "front_mid360")
        msg.timebase = stamp_ns
        msg.lidar_id = 0
        msg.rsvd = [0, 0, 0]

        az = rng.uniform(0.0, 2.0 * math.pi, POINTS_PER_FRAME)
        el = rng.uniform(ELEV_MIN, ELEV_MAX, POINTS_PER_FRAME)
        dirs_body = np.stack([np.cos(el) * np.cos(az),
                              np.cos(el) * np.sin(az),
                              np.sin(el)], axis=1)
        offsets = (np.arange(POINTS_PER_FRAME) + 1.0) / POINTS_PER_FRAME * frame_dt
        keep = rng.random(POINTS_PER_FRAME) > NO_RETURN_P

        points = []
        for i in range(POINTS_PER_FRAME):
            if not keep[i]:
                continue
            t_point = t_frame + offsets[i]
            origin = position(t_point)
            rot = rot_z(yaw(t_point))
            dist = cast_ray(origin, rot @ dirs_body[i])
            if dist is None or dist > 60.0 or dist < 0.5:
                continue
            local = dist * dirs_body[i]
            norm = float(np.linalg.norm(local))
            local = local * ((norm + rng.normal(0.0, RANGE_SIGMA)) / norm)
            p = CustomPoint()
            p.offset_time = int(round(offsets[i] * 1e9))
            p.x = float(local[0])
            p.y = float(local[1])
            p.z = float(local[2])
            p.reflectivity = int(np.clip(rng.normal(100.0, 10.0), 1, 255))
            p.tag = 0x10
            p.line = i % 4
            points.append(p)

        msg.point_num = len(points)
        msg.points = points
        total_points += len(points)
        writer.write("/livox/lidar", rclpy.serialization.serialize_message(msg), stamp_ns)

    del writer
    print(f"lidar 帧数: {n_frames}, IMU 帧数: {n_imu}, 点总数: {total_points}")
    print(f"bag: {BAG_URI}")


if __name__ == "__main__":
    main()
