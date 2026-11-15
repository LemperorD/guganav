#!/usr/bin/env python3
# Copyright 2026 guganav
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""启动阶段看门狗：保证 small_gicp_relocalization 最终进入 active。

Nav2 的 lifecycle_manager 通过 change_state 服务驱动 small_gicp_relocalization。
服务回执偶尔丢失时，节点会停在 inactive（或 unconfigured），manager 则一直停
在 Configuring / Activating。节点不激活就不会建立 map -> odom 的 TF 定时器，
global_costmap 会一直等待 map 坐标系，表现为地图层整块不出现。

本节点只处理启动阶段：某个状态持续 stuck_sec 秒仍未前进，就补发 configure
或 activate。节点一旦进入 active 立即退出，之后的 LOST 自停用不受本节点影响。
"""

import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class RelocalizationWatchdog(Node):
    def __init__(self):
        super().__init__("relocalization_watchdog")

        self.target_node = str(
            self.declare_parameter("target_node", "small_gicp_relocalization").value
        ).rstrip("/")
        self.stuck_sec = float(self.declare_parameter("stuck_sec", 5.0).value)
        self.give_up_sec = float(self.declare_parameter("give_up_sec", 60.0).value)
        self.poll_sec = float(self.declare_parameter("poll_sec", 0.5).value)

        # 相对名称随节点命名空间解析，实车与仿真的 namespace 都能对上
        self.get_state_client = self.create_client(
            GetState, f"{self.target_node}/get_state"
        )
        self.change_state_client = self.create_client(
            ChangeState, f"{self.target_node}/change_state"
        )

        self.service_executor = SingleThreadedExecutor()
        self.service_executor.add_node(self)
        self.last_state = None
        self.state_since = None

    def _call(self, client, request, timeout_sec):
        """同步调用服务；服务不可达或超时返回 None，不抛异常。"""
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                return None
        future = client.call_async(request)
        self.service_executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as exc:  # rclpy 在服务端异常时会在这里抛出
            self.get_logger().warn(f"调用 {client.srv_name} 失败：{exc}")
            return None

    def _query_state(self):
        response = self._call(self.get_state_client, GetState.Request(), 1.0)
        if response is None:
            return None
        return int(response.current_state.id)

    def _change_state(self, transition, name):
        request = ChangeState.Request()
        request.transition.id = transition
        response = self._call(self.change_state_client, request, 5.0)
        if response is None:
            self.get_logger().warn(f"补发 {name} 没有收到回执，稍后重试")
            return False
        if not response.success:
            self.get_logger().warn(f"补发 {name} 被拒绝，当前状态不允许该转移")
            return False
        self.get_logger().warn(f"补发 {name} 成功")
        return True

    def run(self):
        start = time.monotonic()
        self.get_logger().info(
            f"看门狗已启动，监视 {self.target_node}，"
            f"状态停滞超过 {self.stuck_sec:.0f} 秒就补发转移"
        )

        while rclpy.ok():
            now = time.monotonic()
            if now - start > self.give_up_sec:
                self.get_logger().error(
                    f"等待 {self.target_node} 进入 active 超时"
                    f"（{self.give_up_sec:.0f} 秒）。先检查 prior_pcd_file "
                    "指向的先验点云是否存在、是否与当前地图配套"
                )
                return 1

            state = self._query_state()
            if state is None:
                time.sleep(self.poll_sec)
                continue

            if state != self.last_state:
                self.last_state = state
                self.state_since = now
                if state == State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info(
                        f"{self.target_node} 已进入 active，看门狗退出"
                    )
                    return 0
                if state == State.PRIMARY_STATE_FINALIZED:
                    self.get_logger().error(
                        f"{self.target_node} 进入 finalized，通常是配置失败，"
                        "先检查 prior_pcd_file 指向的文件是否可读"
                    )
                    return 1
                if state in (
                    State.PRIMARY_STATE_UNCONFIGURED,
                    State.PRIMARY_STATE_INACTIVE,
                ):
                    self.get_logger().info(
                        f"{self.target_node} 当前状态 {state}，"
                        f"{self.stuck_sec:.0f} 秒内没有前进就补发转移"
                    )
            elif (
                state
                in (
                    State.PRIMARY_STATE_UNCONFIGURED,
                    State.PRIMARY_STATE_INACTIVE,
                )
                and now - self.state_since > self.stuck_sec
            ):
                if state == State.PRIMARY_STATE_UNCONFIGURED:
                    self._change_state(Transition.TRANSITION_CONFIGURE, "configure")
                else:
                    self._change_state(Transition.TRANSITION_ACTIVATE, "activate")
                # 重置计时，避免对同一个停滞状态连续补发
                self.state_since = time.monotonic()

            time.sleep(self.poll_sec)

        return 1


def main():
    rclpy.init()
    node = RelocalizationWatchdog()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
