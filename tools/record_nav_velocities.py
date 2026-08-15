#!/usr/bin/env python3
"""Record the three velocity stages used by the navigation stack.

Example:
  python3 tools/record_nav_velocities.py --mode pid --output /tmp/pid.csv
  python3 tools/record_nav_velocities.py --mode mppi --output /tmp/mppi.csv
"""

import argparse
import csv
import datetime as dt
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


TOPIC_SUFFIXES = (
    "cmd_vel_controller",
    "cmd_vel_nav2_result",
    "cmd_vel",
)


def namespaced_topic(namespace: str, suffix: str) -> str:
    namespace = namespace.strip("/")
    return f"/{namespace}/{suffix}" if namespace else f"/{suffix}"


class VelocityRecorder(Node):
    def __init__(self, mode: str, namespace: str, output: pathlib.Path, duration: float):
        super().__init__("nav_velocity_recorder")
        self.mode = mode
        self.start_monotonic = time.monotonic()
        self.duration = duration
        self.counts = {suffix: 0 for suffix in TOPIC_SUFFIXES}
        self.output = output
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "mode",
                "topic",
                "wall_time_ns",
                "elapsed_s",
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ]
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        for suffix in TOPIC_SUFFIXES:
            topic = namespaced_topic(namespace, suffix)
            self.create_subscription(
                Twist,
                topic,
                lambda msg, topic=topic, suffix=suffix: self.record(topic, suffix, msg),
                qos,
            )
            self.get_logger().info(f"Recording {topic}")

        if duration > 0.0:
            self.create_timer(0.1, self.check_duration)

    def record(self, topic: str, suffix: str, msg: Twist) -> None:
        now_ns = time.time_ns()
        self.writer.writerow(
            [
                self.mode,
                topic,
                now_ns,
                f"{time.monotonic() - self.start_monotonic:.6f}",
                f"{msg.linear.x:.9f}",
                f"{msg.linear.y:.9f}",
                f"{msg.linear.z:.9f}",
                f"{msg.angular.x:.9f}",
                f"{msg.angular.y:.9f}",
                f"{msg.angular.z:.9f}",
            ]
        )
        self.file.flush()
        self.counts[suffix] += 1

    def check_duration(self) -> None:
        if time.monotonic() - self.start_monotonic >= self.duration:
            self.get_logger().info("Recording duration reached")
            rclpy.shutdown()

    def close(self) -> None:
        if not self.file.closed:
            self.file.close()
        summary = ", ".join(f"{topic}={self.counts[topic]}" for topic in TOPIC_SUFFIXES)
        self.get_logger().info(f"Saved {self.output} ({summary})")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("pid", "mppi"), required=True)
    parser.add_argument("--namespace", default="red_standard_robot1")
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to record; 0 records until Ctrl-C",
    )
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must be >= 0")

    if args.output is None:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = pathlib.Path(f"/tmp/nav_vel_{args.mode}_{stamp}.csv")

    rclpy.init()
    recorder = VelocityRecorder(args.mode, args.namespace, args.output, args.duration)
    try:
        rclpy.spin(recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        recorder.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
