#!/usr/bin/env python3
"""启动 semantic_map 节点。

隧道参数来自 yaml 文件，由本文件读取并校验，再作为 ROS 参数传给节点。
校验失败直接让 launch 失败，避免半套参数进到导航里。
"""

import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _fail(message: str) -> None:
    raise RuntimeError(f"[semantic_map] {message}")


def _require_number(value, name: str, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(f"{path} 的 {name} 必须是数字，实际是 {value!r}")
    return float(value)


def _require_point(value, name: str, path: str):
    if not isinstance(value, list) or len(value) != 2:
        _fail(f"{path} 的 {name} 必须是两个数，实际是 {value!r}")
    return [
        _require_number(value[0], f"{name}[0]", path),
        _require_number(value[1], f"{name}[1]", path),
    ]


def load_tunnel(path: str) -> dict:
    """读 yaml 并校验，返回可直接作为 ROS 参数使用的字典。"""
    if not os.path.isfile(path):
        _fail(f"找不到隧道文件: {path}")

    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        _fail(f"{path} 顶层必须是映射")

    p0 = _require_point(data.get("p0"), "p0", path)
    p1 = _require_point(data.get("p1"), "p1", path)
    half_width = _require_number(data.get("half_width"), "half_width", path)
    approach_margin = _require_number(
        data.get("approach_margin", 0.0), "approach_margin", path)

    if half_width <= 0.0:
        _fail(f"{path} 的 half_width 必须为正，实际是 {half_width}")
    if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) <= 1e-6:
        _fail(f"{path} 的 p0 与 p1 重合，隧道长度为零")
    if approach_margin < 0.0:
        _fail(f"{path} 的 approach_margin 不能为负，实际是 {approach_margin}")

    return {
        "p0": p0,
        "p1": p1,
        "half_width": half_width,
        "approach_margin": approach_margin,
    }


def _launch_node(context, *args, **kwargs):
    path = LaunchConfiguration("tunnel_yaml").perform(context)
    parameters = load_tunnel(path)
    return [
        Node(
            package="guga_decision_common",
            executable="semantic_map_server",
            name="semantic_map",
            parameters=[parameters],
            output="screen",
        )
    ]


def generate_launch_description():
    default_yaml = os.path.join(
        get_package_share_directory("guga_bringup"),
        "map", "simulation", "rmul_2027_tunnel.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "tunnel_yaml",
            default_value=default_yaml,
            description="隧道 yaml 文件路径",
        ),
        OpaqueFunction(function=_launch_node),
    ])
