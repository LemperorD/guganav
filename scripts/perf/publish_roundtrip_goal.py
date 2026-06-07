#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped


def parse_args():
    parser = argparse.ArgumentParser(
        description="按固定间隔在两个点之间往返发布 goal_pose。"
    )
    parser.add_argument("--topic", default="goal_pose", help="goal topic，默认: goal_pose")
    parser.add_argument("--frame", default="map", help="frame_id，默认: map")
    parser.add_argument("--namespace", default="", help="topic namespace，例如 /red_standard_robot1")
    parser.add_argument("--simulation", action="store_true", help="使用仿真默认 namespace: /red_standard_robot1")
    parser.add_argument("--interval", type=float, default=30.0, help="发布间隔秒数，默认: 30")
    parser.add_argument("--count", type=int, default=0, help="发布次数，0 表示无限循环")
    parser.add_argument("--start", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "YAW"), help="起点 x y yaw，默认: 0 0 0")
    parser.add_argument("--end", nargs=3, type=float, default=[9.0, -5.5, 0.0], metavar=("X", "Y", "YAW"), help="终点 x y yaw，默认: 9 -5.5 0")
    return parser.parse_args()


def normalize_namespace(namespace):
    if not namespace:
        return ""
    namespace = "/" + namespace.lstrip("/")
    return namespace.rstrip("/")


def resolve_topic(topic, namespace):
    if topic.startswith("/"):
        base = topic
    else:
        base = "/" + topic
    if not namespace:
        return base
    return namespace + base


def make_goal(node, frame, point):
    x, y, yaw = point
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def main():
    args = parse_args()
    namespace = "/red_standard_robot1" if args.simulation else normalize_namespace(args.namespace)
    topic = resolve_topic(args.topic, namespace)

    rclpy.init()
    node = rclpy.create_node("roundtrip_goal_publisher")
    publisher = node.create_publisher(PoseStamped, topic, 10)

    points = [args.start, args.end]
    published = 0

    node.get_logger().info(
        f"publish roundtrip goal: topic={topic}, frame={args.frame}, "
        f"interval={args.interval}s, count={args.count or 'inf'}"
    )

    try:
        while rclpy.ok() and (args.count == 0 or published < args.count):
            point = points[published % 2]
            goal = make_goal(node, args.frame, point)
            publisher.publish(goal)
            node.get_logger().info(
                f"goal[{published + 1}]: x={point[0]:.3f}, y={point[1]:.3f}, yaw={point[2]:.3f}"
            )
            published += 1
            if args.count != 0 and published >= args.count:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
