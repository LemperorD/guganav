"""ROS 2 node that records robot-side navigation evaluation data."""

from __future__ import annotations

import csv
import json
import math
import os
import platform
import socket
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from guga_evaluate.metrics import (
    RisingEdgeCounter,
    SeriesStats,
    TopicTiming,
    nearest_path_error,
    path_curvatures,
    path_length,
    quaternion_to_yaw,
    wrap_angle,
)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@dataclass
class PoseSample:
    stamp: float
    frame_id: str
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass
class PlanCache:
    frame_id: str
    points: list[tuple[float, float]]


@dataclass
class GoalState:
    goal_id: int
    received_at: float
    frame_id: str
    x: float
    y: float
    yaw: float
    candidate_since: Optional[float] = None
    reached_at: Optional[float] = None
    failed_at: Optional[float] = None
    failure_reason: Optional[str] = None


class CsvSink:
    def __init__(self, path: Path, fieldnames: list[str]) -> None:
        self._stream = path.open("w", encoding="utf-8", newline="", buffering=1)
        self._writer = csv.DictWriter(self._stream, fieldnames=fieldnames)
        self._writer.writeheader()

    def write(self, **row) -> None:
        self._writer.writerow(row)

    def close(self) -> None:
        self._stream.close()


class NullSink:
    """Drop rows when recording is disabled."""

    def write(self, **row) -> None:
        del row

    def close(self) -> None:
        pass


class EvaluateNode(Node):
    def __init__(self) -> None:
        super().__init__("guga_evaluate")
        self._declare_parameters()
        self._load_parameters()

        self.output_dir = Path(os.path.expanduser(self.output_dir)).resolve()
        if self.save_data:
            self.output_dir.mkdir(parents=True, exist_ok=True)
        self._closed = False
        self._warned_frame_pairs: set[tuple[str, str]] = set()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.topic_timing = {
            "cmd_vel": TopicTiming(),
            "odometry": TopicTiming(),
            "local_plan": TopicTiming(),
            "global_plan": TopicTiming(),
            "predicted_plan": TopicTiming(),
            "goal": TopicTiming(),
            "ground_truth": TopicTiming(),
            "collision": TopicTiming(),
            "emergency_stop": TopicTiming(),
        }
        self.speed_error_linear = SeriesStats()
        self.speed_error_vx = SeriesStats()
        self.speed_error_vy = SeriesStats()
        self.speed_error_wz = SeriesStats()
        self.cross_track_error = SeriesStats()
        self.heading_error = SeriesStats()
        self.goal_distance = SeriesStats()
        self.goal_yaw_error = SeriesStats()
        self.command_acceleration = SeriesStats()
        self.command_angular_acceleration = SeriesStats()
        self.command_jerk = SeriesStats()
        self.gt_position_error = SeriesStats()
        self.gt_yaw_error = SeriesStats()
        self.time_to_reach = SeriesStats()
        self.collision_events = RisingEdgeCounter(
            debounce_sec=float(self.safety_event_debounce_sec))
        self.emergency_stop_events = RisingEdgeCounter(
            debounce_sec=float(self.safety_event_debounce_sec))
        self.plan_metrics: dict[str, list[dict]] = {
            "local": [], "global": [], "predicted": []
        }

        self.latest_cmd: Optional[tuple[float, float, float, float]] = None
        self.previous_cmd: Optional[tuple[float, float, float, float]] = None
        self.previous_cmd_acceleration: Optional[tuple[float, float, float]] = None
        self.previous_pose: Optional[PoseSample] = None
        self.filtered_velocity: Optional[tuple[float, float, float]] = None
        self.latest_ground_truth: Optional[PoseSample] = None
        self.local_plan: Optional[PlanCache] = None
        self.global_plan: Optional[PlanCache] = None
        self.predicted_plan: Optional[PlanCache] = None
        self.current_goal: Optional[GoalState] = None
        self.goal_history: list[GoalState] = []
        self.goal_sequence = 0
        self.live_command: dict = {}
        self.live_actual: dict = {}
        self.live_tracking: dict = {}

        self._open_csv_files()
        if self.save_data:
            self._write_metadata()
        self._create_subscriptions()
        self.metrics_publisher = self.create_publisher(
            String, self.metrics_topic, 10)
        self.publish_timer = self.create_timer(
            self.publish_period_sec, self.publish_live_metrics)
        self.summary_timer = None
        if self.save_data:
            self.summary_timer = self.create_timer(
                self.summary_period_sec, self.write_summary)

        self.get_logger().info(
            f"Evaluation started. recording={'on' if self.save_data else 'off'} "
            f"output={self.output_dir if self.save_data else '-'} "
            f"ground_truth={self.use_ground_truth}")

    def _declare_parameters(self) -> None:
        self.declare_parameter("output_dir", "/tmp/guga_evaluate")
        self.declare_parameter("save_data", False)
        self.declare_parameter("mode", "reality")
        self.declare_parameter("workspace", "")
        self.declare_parameter("metrics_topic", "evaluation_metrics")
        self.declare_parameter("publish_period_sec", 0.1)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("odometry_topic", "odometry")
        self.declare_parameter("local_plan_topic", "local_plan")
        self.declare_parameter("global_plan_topic", "plan")
        self.declare_parameter("predicted_plan_topic", "predicted_plan")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("ground_truth_topic", "chassis_odometry_gt")
        self.declare_parameter("collision_topic", "collision_detected")
        self.declare_parameter("emergency_stop_topic", "emergency_stop")
        self.declare_parameter("use_ground_truth", False)
        self.declare_parameter("use_collision_topic", False)
        self.declare_parameter("use_emergency_stop_topic", False)
        self.declare_parameter("safety_event_debounce_sec", 0.5)
        self.declare_parameter("summary_period_sec", 5.0)
        self.declare_parameter("max_cmd_age_sec", 0.5)
        self.declare_parameter("max_ground_truth_age_sec", 0.2)
        self.declare_parameter("min_pose_dt_sec", 0.005)
        self.declare_parameter("max_pose_dt_sec", 0.5)
        self.declare_parameter("velocity_filter_alpha", 0.35)
        self.declare_parameter("goal_xy_tolerance", 0.15)
        self.declare_parameter("goal_yaw_tolerance", 0.15)
        self.declare_parameter("goal_speed_tolerance", 0.10)
        self.declare_parameter("goal_dwell_sec", 0.5)
        self.declare_parameter("goal_dedup_xy_tolerance", 0.01)
        self.declare_parameter("goal_dedup_yaw_tolerance", 0.01)
        self.declare_parameter("goal_timeout_sec", 0.0)

    def _load_parameters(self) -> None:
        for name in (
            "output_dir", "save_data", "mode", "workspace", "metrics_topic",
            "publish_period_sec", "cmd_vel_topic",
            "odometry_topic", "local_plan_topic", "global_plan_topic",
            "predicted_plan_topic", "goal_topic", "ground_truth_topic",
            "collision_topic", "emergency_stop_topic", "use_ground_truth",
            "use_collision_topic", "use_emergency_stop_topic",
            "safety_event_debounce_sec", "summary_period_sec", "max_cmd_age_sec",
            "max_ground_truth_age_sec", "min_pose_dt_sec", "max_pose_dt_sec",
            "velocity_filter_alpha", "goal_xy_tolerance",
            "goal_yaw_tolerance", "goal_speed_tolerance", "goal_dwell_sec",
            "goal_dedup_xy_tolerance", "goal_dedup_yaw_tolerance",
            "goal_timeout_sec",
        ):
            setattr(self, name, self.get_parameter(name).value)

    def _open_csv_files(self) -> None:
        if not self.save_data:
            null_sink = NullSink()
            self.state_csv = null_sink
            self.command_csv = null_sink
            self.tracking_csv = null_sink
            self.plan_csv = null_sink
            self.event_csv = null_sink
            return
        self.state_csv = CsvSink(self.output_dir / "state.csv", [
            "receipt_time_sec", "header_time_sec", "frame_id", "child_frame_id",
            "x", "y", "yaw", "reported_vx", "reported_vy", "reported_wz",
            "derived_vx", "derived_vy", "derived_wz",
        ])
        self.command_csv = CsvSink(self.output_dir / "command.csv", [
            "receipt_time_sec", "vx", "vy", "wz", "linear_acceleration",
            "angular_acceleration", "jerk",
        ])
        self.tracking_csv = CsvSink(self.output_dir / "tracking.csv", [
            "time_sec", "path_source", "cross_track_error_m",
            "heading_error_rad", "path_progress", "goal_distance_m",
            "goal_yaw_error_rad", "speed_error_mps", "wz_error_radps",
            "gt_position_error_m", "gt_yaw_error_rad",
        ])
        self.plan_csv = CsvSink(self.output_dir / "plans.csv", [
            "receipt_time_sec", "source", "header_time_sec", "frame_id",
            "point_count", "length_m", "mean_curvature_inv_m",
            "max_curvature_inv_m",
        ])
        self.event_csv = CsvSink(self.output_dir / "events.csv", [
            "time_sec", "event", "goal_id", "value",
        ])

    def _write_metadata(self) -> None:
        metadata = {
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "mode": self.mode,
            "namespace": self.get_namespace(),
            "node": self.get_fully_qualified_name(),
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "python": sys.version,
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "workspace": self.workspace,
            "parameters": {
                name: self.get_parameter(name).value
                for name in self._parameters
                if name != "use_sim_time"
            },
        }
        self._write_json_atomic(self.output_dir / "metadata.json", metadata)

    def _create_subscriptions(self) -> None:
        sensor_qos = QoSProfile(depth=100)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        reliable_qos = QoSProfile(depth=20)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            Twist, self.cmd_vel_topic, self._on_cmd_vel, reliable_qos)
        self.create_subscription(
            Odometry, self.odometry_topic, self._on_odometry, sensor_qos)
        self.create_subscription(
            PathMsg, self.local_plan_topic,
            lambda msg: self._on_plan("local", msg), reliable_qos)
        self.create_subscription(
            PathMsg, self.global_plan_topic,
            lambda msg: self._on_plan("global", msg), reliable_qos)
        self.create_subscription(
            PathMsg, self.predicted_plan_topic,
            lambda msg: self._on_plan("predicted", msg), reliable_qos)
        self.create_subscription(
            PoseStamped, self.goal_topic, self._on_goal, reliable_qos)
        if self.use_ground_truth:
            self.create_subscription(
                Odometry, self.ground_truth_topic,
                self._on_ground_truth, sensor_qos)
        if self.use_collision_topic:
            self.create_subscription(
                Bool, self.collision_topic,
                self._on_collision_state, reliable_qos)
        if self.use_emergency_stop_topic:
            self.create_subscription(
                Bool, self.emergency_stop_topic,
                self._on_emergency_stop_state, reliable_qos)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = self._now()
        self.topic_timing["cmd_vel"].observe(now)
        acceleration = math.nan
        angular_acceleration = math.nan
        jerk = math.nan
        if self.previous_cmd is not None:
            dt = now - self.previous_cmd[0]
            if dt > 1e-6:
                ax = (msg.linear.x - self.previous_cmd[1]) / dt
                ay = (msg.linear.y - self.previous_cmd[2]) / dt
                angular_acceleration = (msg.angular.z - self.previous_cmd[3]) / dt
                acceleration = math.hypot(ax, ay)
                self.command_acceleration.add(acceleration)
                self.command_angular_acceleration.add(abs(angular_acceleration))
                if self.previous_cmd_acceleration is not None:
                    prev_time, prev_acc, _ = self.previous_cmd_acceleration
                    acc_dt = now - prev_time
                    if acc_dt > 1e-6:
                        jerk = abs(acceleration - prev_acc) / acc_dt
                        self.command_jerk.add(jerk)
                self.previous_cmd_acceleration = (
                    now, acceleration, angular_acceleration)
        self.previous_cmd = (now, msg.linear.x, msg.linear.y, msg.angular.z)
        self.latest_cmd = self.previous_cmd
        self.live_command = {
            "vx_mps": self._finite_or_none(msg.linear.x),
            "vy_mps": self._finite_or_none(msg.linear.y),
            "wz_radps": self._finite_or_none(msg.angular.z),
            "linear_speed_mps": self._finite_or_none(
                math.hypot(msg.linear.x, msg.linear.y)),
        }
        self.command_csv.write(
            receipt_time_sec=now, vx=msg.linear.x, vy=msg.linear.y,
            wz=msg.angular.z, linear_acceleration=acceleration,
            angular_acceleration=angular_acceleration, jerk=jerk)

    def _odom_sample(self, msg: Odometry) -> PoseSample:
        pose = msg.pose.pose
        twist = msg.twist.twist
        header_time = stamp_seconds(msg.header.stamp)
        return PoseSample(
            stamp=header_time if header_time > 0.0 else self._now(),
            frame_id=msg.header.frame_id,
            x=pose.position.x,
            y=pose.position.y,
            yaw=quaternion_to_yaw(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w),
            vx=twist.linear.x,
            vy=twist.linear.y,
            wz=twist.angular.z,
        )

    def _derived_velocity(
        self, sample: PoseSample
    ) -> Optional[tuple[float, float, float]]:
        if self.previous_pose is None:
            self.previous_pose = sample
            return None
        dt = sample.stamp - self.previous_pose.stamp
        if not self.min_pose_dt_sec <= dt <= self.max_pose_dt_sec:
            self.previous_pose = sample
            return None
        dx = sample.x - self.previous_pose.x
        dy = sample.y - self.previous_pose.y
        yaw = self.previous_pose.yaw
        raw = (
            (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt,
            (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dt,
            wrap_angle(sample.yaw - self.previous_pose.yaw) / dt,
        )
        self.previous_pose = sample
        alpha = max(0.0, min(1.0, float(self.velocity_filter_alpha)))
        if self.filtered_velocity is None:
            self.filtered_velocity = raw
        else:
            self.filtered_velocity = tuple(
                alpha * value + (1.0 - alpha) * previous
                for value, previous in zip(raw, self.filtered_velocity)
            )
        return self.filtered_velocity

    def _on_odometry(self, msg: Odometry) -> None:
        now = self._now()
        header_time = stamp_seconds(msg.header.stamp)
        self.topic_timing["odometry"].observe(now, header_time)
        sample = self._odom_sample(msg)
        derived = self._derived_velocity(sample)
        dvx, dvy, dwz = derived if derived is not None else (
            math.nan, math.nan, math.nan)
        actual = derived if derived is not None else (
            sample.vx, sample.vy, sample.wz)
        self.live_actual = {
            "vx_mps": self._finite_or_none(actual[0]),
            "vy_mps": self._finite_or_none(actual[1]),
            "wz_radps": self._finite_or_none(actual[2]),
            "linear_speed_mps": self._finite_or_none(
                math.hypot(actual[0], actual[1])),
            "source": "pose_difference" if derived is not None else "odometry",
        }
        self.state_csv.write(
            receipt_time_sec=now, header_time_sec=header_time,
            frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id,
            x=sample.x, y=sample.y, yaw=sample.yaw,
            reported_vx=sample.vx, reported_vy=sample.vy,
            reported_wz=sample.wz, derived_vx=dvx, derived_vy=dvy,
            derived_wz=dwz)
        self._evaluate_tracking(sample, derived, now)

    def _on_ground_truth(self, msg: Odometry) -> None:
        now = self._now()
        header_time = stamp_seconds(msg.header.stamp)
        self.topic_timing["ground_truth"].observe(now, header_time)
        self.latest_ground_truth = self._odom_sample(msg)

    def _on_collision_state(self, msg: Bool) -> None:
        now = self._now()
        self.topic_timing["collision"].observe(now)
        if self.collision_events.observe(msg.data, now):
            self.event_csv.write(
                time_sec=now, event="collision", goal_id=self._goal_id(),
                value=self.collision_events.count)

    def _on_emergency_stop_state(self, msg: Bool) -> None:
        now = self._now()
        self.topic_timing["emergency_stop"].observe(now)
        if self.emergency_stop_events.observe(msg.data, now):
            self.event_csv.write(
                time_sec=now, event="emergency_stop",
                goal_id=self._goal_id(),
                value=self.emergency_stop_events.count)

    def _goal_id(self):
        return self.current_goal.goal_id if self.current_goal else ""

    def _on_plan(self, source: str, msg: PathMsg) -> None:
        now = self._now()
        header_time = stamp_seconds(msg.header.stamp)
        self.topic_timing[f"{source}_plan"].observe(now, header_time)
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in msg.poses]
        cache = PlanCache(msg.header.frame_id, points)
        setattr(self, f"{source}_plan", cache)
        length = path_length(points)
        curvatures = path_curvatures(points)
        metrics = {
            "point_count": len(points),
            "length_m": length,
            "mean_curvature_inv_m": (
                sum(curvatures) / len(curvatures) if curvatures else 0.0),
            "max_curvature_inv_m": max(curvatures) if curvatures else 0.0,
        }
        self.plan_metrics[source].append(metrics)
        self.plan_csv.write(
            receipt_time_sec=now, source=source,
            header_time_sec=header_time, frame_id=msg.header.frame_id,
            **metrics)

    def _on_goal(self, msg: PoseStamped) -> None:
        now = self._now()
        header_time = stamp_seconds(msg.header.stamp)
        self.topic_timing["goal"].observe(now, header_time)
        orientation = msg.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w)
        if self.current_goal is not None:
            same_frame = self.current_goal.frame_id == msg.header.frame_id
            position_delta = math.hypot(
                msg.pose.position.x - self.current_goal.x,
                msg.pose.position.y - self.current_goal.y)
            yaw_delta = abs(wrap_angle(yaw - self.current_goal.yaw))
            if (same_frame
                    and position_delta <= self.goal_dedup_xy_tolerance
                    and yaw_delta <= self.goal_dedup_yaw_tolerance
                    and self.current_goal.failed_at is None):
                return

        if self.current_goal is not None:
            self._fail_goal(self.current_goal, now, "superseded")

        self.goal_sequence += 1
        goal = GoalState(
            goal_id=self.goal_sequence,
            received_at=now,
            frame_id=msg.header.frame_id,
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=yaw,
        )
        self.current_goal = goal
        self.goal_history.append(goal)
        self.event_csv.write(
            time_sec=now, event="goal_received", goal_id=goal.goal_id,
            value=f"{goal.x:.6f},{goal.y:.6f},{goal.yaw:.6f}")

    def _transform_pose(
        self, sample: PoseSample, target_frame: str
    ) -> Optional[tuple[float, float, float]]:
        if not target_frame or not sample.frame_id or target_frame == sample.frame_id:
            return sample.x, sample.y, sample.yaw
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, sample.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.02))
        except TransformException as error:
            pair = (sample.frame_id, target_frame)
            if pair not in self._warned_frame_pairs:
                self._warned_frame_pairs.add(pair)
                self.get_logger().warning(
                    "Cannot transform evaluation pose "
                    f"{sample.frame_id} -> {target_frame}: {error}")
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_yaw = quaternion_to_yaw(
            rotation.x, rotation.y, rotation.z, rotation.w)
        x = translation.x + math.cos(transform_yaw) * sample.x \
            - math.sin(transform_yaw) * sample.y
        y = translation.y + math.sin(transform_yaw) * sample.x \
            + math.cos(transform_yaw) * sample.y
        return x, y, wrap_angle(transform_yaw + sample.yaw)

    def _select_plan(self) -> tuple[str, Optional[PlanCache]]:
        if self.local_plan is not None and len(self.local_plan.points) >= 2:
            return "local", self.local_plan
        if self.global_plan is not None and len(self.global_plan.points) >= 2:
            return "global", self.global_plan
        return "", None

    def _evaluate_tracking(
        self,
        sample: PoseSample,
        derived_velocity: Optional[tuple[float, float, float]],
        now: float,
    ) -> None:
        path_source, plan = self._select_plan()
        cross_track = heading = progress = math.nan
        if plan is not None:
            transformed = self._transform_pose(sample, plan.frame_id)
            if transformed is not None:
                path_error = nearest_path_error(*transformed, plan.points)
                if path_error is not None:
                    cross_track, heading, progress = path_error
                    self.cross_track_error.add(cross_track)
                    self.heading_error.add(abs(heading))

        goal_distance = goal_yaw = math.nan
        if self.current_goal is not None:
            transformed = self._transform_pose(sample, self.current_goal.frame_id)
            if transformed is not None:
                goal_distance = math.hypot(
                    transformed[0] - self.current_goal.x,
                    transformed[1] - self.current_goal.y)
                goal_yaw = abs(wrap_angle(transformed[2] - self.current_goal.yaw))
                self.goal_distance.add(goal_distance)
                self.goal_yaw_error.add(goal_yaw)
                self._update_goal_state(
                    now, goal_distance, goal_yaw, derived_velocity)

        speed_error = wz_error = math.nan
        if derived_velocity is not None and self.latest_cmd is not None:
            cmd_age = now - self.latest_cmd[0]
            if 0.0 <= cmd_age <= self.max_cmd_age_sec:
                evx = derived_velocity[0] - self.latest_cmd[1]
                evy = derived_velocity[1] - self.latest_cmd[2]
                ewz = derived_velocity[2] - self.latest_cmd[3]
                speed_error = math.hypot(evx, evy)
                wz_error = ewz
                self.speed_error_vx.add(evx)
                self.speed_error_vy.add(evy)
                self.speed_error_linear.add(speed_error)
                self.speed_error_wz.add(ewz)

        gt_position = gt_yaw = math.nan
        if self.latest_ground_truth is not None:
            age = abs(sample.stamp - self.latest_ground_truth.stamp)
            if (age <= self.max_ground_truth_age_sec
                    and sample.frame_id == self.latest_ground_truth.frame_id):
                gt_position = math.hypot(
                    sample.x - self.latest_ground_truth.x,
                    sample.y - self.latest_ground_truth.y)
                gt_yaw = abs(wrap_angle(
                    sample.yaw - self.latest_ground_truth.yaw))
                self.gt_position_error.add(gt_position)
                self.gt_yaw_error.add(gt_yaw)

        self.tracking_csv.write(
            time_sec=now, path_source=path_source,
            cross_track_error_m=cross_track, heading_error_rad=heading,
            path_progress=progress, goal_distance_m=goal_distance,
            goal_yaw_error_rad=goal_yaw, speed_error_mps=speed_error,
            wz_error_radps=wz_error, gt_position_error_m=gt_position,
            gt_yaw_error_rad=gt_yaw)
        self.live_tracking = {
            "path_source": path_source,
            "cross_track_error_m": self._finite_or_none(cross_track),
            "heading_error_rad": self._finite_or_none(heading),
            "path_progress": self._finite_or_none(progress),
            "goal_distance_m": self._finite_or_none(goal_distance),
            "goal_yaw_error_rad": self._finite_or_none(goal_yaw),
            "speed_error_mps": self._finite_or_none(speed_error),
            "wz_error_radps": self._finite_or_none(wz_error),
            "gt_position_error_m": self._finite_or_none(gt_position),
            "gt_yaw_error_rad": self._finite_or_none(gt_yaw),
        }

    def _update_goal_state(
        self,
        now: float,
        distance: float,
        yaw_error: float,
        velocity: Optional[tuple[float, float, float]],
    ) -> None:
        goal = self.current_goal
        if (goal is None or goal.reached_at is not None
                or goal.failed_at is not None):
            return
        speed = math.inf
        if velocity is not None:
            speed = math.hypot(velocity[0], velocity[1])
        within = (
            distance <= self.goal_xy_tolerance
            and yaw_error <= self.goal_yaw_tolerance
            and speed <= self.goal_speed_tolerance
        )
        if not within:
            goal.candidate_since = None
            return
        if goal.candidate_since is None:
            goal.candidate_since = now
            return
        if now - goal.candidate_since >= self.goal_dwell_sec:
            goal.reached_at = now
            duration = now - goal.received_at
            self.time_to_reach.add(duration)
            self.event_csv.write(
                time_sec=now, event="goal_reached", goal_id=goal.goal_id,
                value=f"{duration:.6f}")

    def _fail_goal(self, goal: GoalState, now: float, reason: str) -> None:
        if goal.reached_at is not None or goal.failed_at is not None:
            return
        goal.failed_at = now
        goal.failure_reason = reason
        self.event_csv.write(
            time_sec=now, event="goal_failed", goal_id=goal.goal_id,
            value=reason)

    def _check_goal_timeout(self, now: float) -> None:
        goal = self.current_goal
        if (goal is None or goal.reached_at is not None
                or goal.failed_at is not None):
            return
        if (self.goal_timeout_sec > 0.0
                and now - goal.received_at >= self.goal_timeout_sec):
            self._fail_goal(goal, now, "timeout")

    @staticmethod
    def _aggregate_plan_metrics(items: list[dict]) -> dict:
        if not items:
            return {"count": 0}
        result = {"count": len(items), "latest": items[-1]}
        for key in (
            "point_count", "length_m", "mean_curvature_inv_m",
            "max_curvature_inv_m",
        ):
            values = [float(item[key]) for item in items]
            result[key] = SeriesStats(values=values).summary()
        return result

    def summary(self) -> dict:
        self._check_goal_timeout(self._now())
        goals = []
        for goal in self.goal_history:
            if goal.reached_at is not None:
                status = "reached"
            elif goal.failed_at is not None:
                status = "failed"
            else:
                status = "active"
            goals.append({
                "goal_id": goal.goal_id,
                "received_at_sec": goal.received_at,
                "status": status,
                "reached": goal.reached_at is not None,
                "reached_at_sec": goal.reached_at,
                "failed_at_sec": goal.failed_at,
                "failure_reason": goal.failure_reason,
                "time_to_reach_sec": (
                    goal.reached_at - goal.received_at
                    if goal.reached_at is not None else None),
                "target": {"x": goal.x, "y": goal.y, "yaw": goal.yaw,
                           "frame_id": goal.frame_id},
            })
        return {
            "updated_at_utc": datetime.now(timezone.utc).isoformat(),
            "mode": self.mode,
            "save_data": self.save_data,
            "mission": self._mission_summary(),
            "safety": self._safety_summary(),
            "topics": {
                name: timing.summary()
                for name, timing in self.topic_timing.items()
            },
            "control": {
                "linear_speed_error_mps": self.speed_error_linear.summary(),
                "vx_error_mps": self.speed_error_vx.summary(),
                "vy_error_mps": self.speed_error_vy.summary(),
                "wz_error_radps": self.speed_error_wz.summary(),
                "command_acceleration_mps2": self.command_acceleration.summary(),
                "command_angular_acceleration_radps2": (
                    self.command_angular_acceleration.summary()),
                "command_jerk_mps3": self.command_jerk.summary(),
            },
            "tracking": {
                "cross_track_error_m": self.cross_track_error.summary(),
                "heading_error_rad": self.heading_error.summary(),
                "goal_distance_m": self.goal_distance.summary(),
                "goal_yaw_error_rad": self.goal_yaw_error.summary(),
            },
            "localization_against_ground_truth": {
                "position_error_m": self.gt_position_error.summary(),
                "yaw_error_rad": self.gt_yaw_error.summary(),
            },
            "plans": {
                source: self._aggregate_plan_metrics(items)
                for source, items in self.plan_metrics.items()
            },
            "goals": goals,
        }

    def _mission_summary(self) -> dict:
        now = self._now()
        received = len(self.goal_history)
        reached = sum(
            goal.reached_at is not None for goal in self.goal_history)
        failed = sum(
            goal.failed_at is not None for goal in self.goal_history)
        active = received - reached - failed
        completed = reached + failed
        current_status = None
        if self.current_goal is not None:
            if self.current_goal.reached_at is not None:
                current_status = "reached"
            elif self.current_goal.failed_at is not None:
                current_status = "failed"
            else:
                current_status = "active"
        return {
            "goals_received": received,
            "goals_reached": reached,
            "goals_failed": failed,
            "goals_active": active,
            "success_rate": reached / received if received else None,
            "completed_success_rate": (
                reached / completed if completed else None),
            "time_to_reach_sec": self.time_to_reach.summary(),
            "latest_time_to_reach_sec": (
                self.time_to_reach.values[-1]
                if self.time_to_reach.values else None),
            "current_goal_id": self._goal_id() or None,
            "current_goal_status": current_status,
            "current_goal_elapsed_sec": (
                now - self.current_goal.received_at
                if self.current_goal is not None
                and current_status == "active" else None),
        }

    def _safety_summary(self) -> dict:
        collision = self.collision_events.summary(
            bool(self.use_collision_topic))
        emergency_stop = self.emergency_stop_events.summary(
            bool(self.use_emergency_stop_topic))
        collision_free = (
            collision["count"] == 0
            if collision["data_available"] else None)
        emergency_stop_free = (
            emergency_stop["count"] == 0
            if emergency_stop["data_available"] else None)
        return {
            "collision": collision,
            "emergency_stop": emergency_stop,
            "collision_free": collision_free,
            "emergency_stop_free": emergency_stop_free,
            "safe_run": (
                collision_free and emergency_stop_free
                if collision_free is not None
                and emergency_stop_free is not None else None),
        }

    @staticmethod
    def _finite_or_none(value: float):
        return float(value) if math.isfinite(value) else None

    @staticmethod
    def _summary_value(stats: SeriesStats, key: str):
        return stats.summary().get(key)

    def live_snapshot(self) -> dict:
        self._check_goal_timeout(self._now())
        current_goal = self.current_goal
        mission = self._mission_summary()
        safety = self._safety_summary()
        return {
            "time_sec": self._now(),
            "mode": self.mode,
            "save_data": self.save_data,
            "command": self.live_command,
            "actual": self.live_actual,
            "tracking": self.live_tracking,
            "mission": mission,
            "safety": safety,
            "aggregate": {
                "speed_error_rmse_mps": self._summary_value(
                    self.speed_error_linear, "rmse"),
                "cross_track_rmse_m": self._summary_value(
                    self.cross_track_error, "rmse"),
                "heading_error_rmse_rad": self._summary_value(
                    self.heading_error, "rmse"),
                "command_hz": self.topic_timing[
                    "cmd_vel"].summary()["effective_hz"],
                "odometry_hz": self.topic_timing[
                    "odometry"].summary()["effective_hz"],
                "goal_id": current_goal.goal_id if current_goal else None,
                "goal_reached": (
                    current_goal.reached_at is not None
                    if current_goal else None),
            },
        }

    def publish_live_metrics(self) -> None:
        message = String()
        message.data = json.dumps(
            self.live_snapshot(), ensure_ascii=False, allow_nan=False,
            separators=(",", ":"))
        self.metrics_publisher.publish(message)

    @staticmethod
    def _write_json_atomic(path: Path, value: dict) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2,
                      allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)

    def write_summary(self) -> None:
        if self.save_data and not self._closed:
            self._write_json_atomic(
                self.output_dir / "summary.json", self.summary())

    def close(self) -> None:
        if self._closed:
            return
        if self.save_data:
            self.write_summary()
        self._closed = True
        for sink in (
            self.state_csv, self.command_csv, self.tracking_csv,
            self.plan_csv, self.event_csv,
        ):
            sink.close()
        if rclpy.ok() and self.save_data:
            self.get_logger().info(
                f"Evaluation results written to {self.output_dir}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
