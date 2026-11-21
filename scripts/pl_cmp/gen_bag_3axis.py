#!/usr/bin/env python3
"""生成三轴转动的合成 rosbag, 不绑定小车运动。

IMU 绕 roll/pitch/yaw 三轴做不同频率的正弦摆动, 平移为零。
雷达原点固定, 房间点云随姿态转到雷达系, 带帧内运动畸变。
用于验证 bg/ba 三轴的可观测性与收敛。

发布内容:
  /livox/lidar  livox_ros_driver2/msg/CustomMsg  10 Hz
  /livox/imu    sensor_msgs/msg/Imu              200 Hz

IMU 单位约定与 config/mid360.yaml 一致: acc_norm = 1.0 -> 加速度以 g 为单位。
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
SENSOR_Z = 0.5
G_M_S2 = 9.81

IMU_RATE = 200.0
LIDAR_RATE = 10.0
POINTS_PER_FRAME = 10000
ELEV_MIN = math.radians(-25.0)
ELEV_MAX = math.radians(30.0)
RANGE_SIGMA = 0.01
NO_RETURN_P = 0.01
T_START_IMU = 0.0
T_START_LIDAR = 1.0
T_END = 30.0
T_STATIC = 3.0

# 三轴摆动幅度 (rad) 与频率 (Hz), 频率互不相同以便分离零偏
ROLL_A, ROLL_F = 0.35, 0.10
PITCH_A, PITCH_F = 0.26, 0.13
YAW_A, YAW_F = 0.52, 0.07

# 三轴平移幅度 (m) 与频率 (Hz), 频率与转动错开
POS_AX, POS_AY, POS_AZ = 0.35, 0.25, 0.20
POS_FX, POS_FY, POS_FZ = 0.08, 0.11, 0.05

BIAS_GYRO = np.array([0.003, -0.002, 0.004])
BIAS_ACC = np.array([0.010, 0.010, 0.020])
SIGMA_GYRO = 0.002
SIGMA_ACC = 0.005

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
PL_DIR = os.environ.get("PL_CMP_DIR", os.path.join(WS, ".pl_cmp_bench"))
BAG_URI = os.path.join(PL_DIR, "bags", "mid360_3axis")
SEED = 20260924


def roll(t):
    tt = max(0.0, t - T_STATIC)
    return ROLL_A * math.sin(2.0 * math.pi * ROLL_F * tt)


def pitch(t):
    tt = max(0.0, t - T_STATIC)
    return PITCH_A * math.sin(2.0 * math.pi * PITCH_F * tt)


def yaw(t):
    tt = max(0.0, t - T_STATIC)
    return YAW_A * math.sin(2.0 * math.pi * YAW_F * tt)


def rot_xyz(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def attitude(t):
    return rot_xyz(roll(t), pitch(t), yaw(t))


def angular_velocity(t, h=1e-3):
    r0 = attitude(t)
    dp = (attitude(t + h) - attitude(t - h)) / (2.0 * h)
    w = r0.T @ dp
    return np.array([w[2, 1], w[0, 2], w[1, 0]])


def position(t):
    tt = max(0.0, t - T_STATIC)
    return np.array([
        POS_AX * math.sin(2.0 * math.pi * POS_FX * tt),
        POS_AY * math.sin(2.0 * math.pi * POS_FY * tt),
        POS_AZ * math.sin(2.0 * math.pi * POS_FZ * tt),
    ])


def acceleration_world(t):
    tt = max(0.0, t - T_STATIC)
    return np.array([
        -POS_AX * (2.0 * math.pi * POS_FX) ** 2
        * math.sin(2.0 * math.pi * POS_FX * tt),
        -POS_AY * (2.0 * math.pi * POS_FY) ** 2
        * math.sin(2.0 * math.pi * POS_FY * tt),
        -POS_AZ * (2.0 * math.pi * POS_FZ) ** 2
        * math.sin(2.0 * math.pi * POS_FZ * tt),
    ])


def specific_force_body(t):
    r = attitude(t)
    g_world = np.array([0.0, 0.0, -G_M_S2])
    return r.T @ (acceleration_world(t) - g_world)


def ray_plane_distance(origin, direction, axis, value):
    if abs(direction[axis]) < 1e-12:
        return None
    dist = (value - origin[axis]) / direction[axis]
    if dist <= 1e-6:
        return None
    return dist


def cast_ray(origin_world, direction_world):
    best = None
    for axis, value in ((0, ROOM_HALF), (0, -ROOM_HALF), (1, ROOM_HALF),
                        (1, -ROOM_HALF), (2, CEIL_Z), (2, FLOOR_Z)):
        dist = ray_plane_distance(origin_world, direction_world, axis, value)
        if dist is None:
            continue
        hit = origin_world + dist * direction_world
        if axis == 0 and (abs(hit[1]) > ROOM_HALF
                          or not FLOOR_Z <= hit[2] <= CEIL_Z):
            continue
        if axis == 1 and (abs(hit[0]) > ROOM_HALF
                          or not FLOOR_Z <= hit[2] <= CEIL_Z):
            continue
        if axis == 2 and (abs(hit[0]) > ROOM_HALF
                          or abs(hit[1]) > ROOM_HALF):
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
        gyro = (angular_velocity(t) + BIAS_GYRO
                + rng.normal(0.0, SIGMA_GYRO, 3))
        acc = (specific_force_body(t) / G_M_S2 + BIAS_ACC
               + rng.normal(0.0, SIGMA_ACC, 3))
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])
        msg.orientation.w = 1.0
        writer.write("/livox/imu", rclpy.serialization.serialize_message(msg),
                     stamp_ns)

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
        offsets = ((np.arange(POINTS_PER_FRAME) + 1.0) / POINTS_PER_FRAME
                   * frame_dt)
        keep = rng.random(POINTS_PER_FRAME) > NO_RETURN_P

        points = []
        for i in range(POINTS_PER_FRAME):
            if not keep[i]:
                continue
            t_point = t_frame + offsets[i]
            origin = position(t_point) + np.array([0.0, 0.0, SENSOR_Z])
            rot = attitude(t_point)
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
        writer.write("/livox/lidar", rclpy.serialization.serialize_message(msg),
                     stamp_ns)

    del writer
    print(f"lidar 帧数: {n_frames}, IMU 帧数: {n_imu}, 点总数: {total_points}")
    print(f"bag: {BAG_URI}")


if __name__ == "__main__":
    main()
