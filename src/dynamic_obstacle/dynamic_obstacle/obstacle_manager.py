#!/usr/bin/env python3

import os
import subprocess
import time
import math

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class ObstacleManager(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_node')

        # =========================
        # ROS 2 parameters
        # =========================

        self.declare_parameter('world_name', 'default')
        self.declare_parameter('obstacle_name', 'dynamic_obstacle')
        self.declare_parameter('model_file', 'obstacle.sdf')

        self.declare_parameter('start_x', -2.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 5.0)

        self.declare_parameter('min_x', -2.0)
        self.declare_parameter('max_x', 2.0)

        self.declare_parameter('speed', 0.5)
        self.declare_parameter('update_rate', 10.0)

        self.declare_parameter('auto_spawn', True)

        # =========================
        # Get parameters
        # =========================

        self.world_name = self.get_parameter(
            'world_name'
        ).value

        self.obstacle_name = self.get_parameter(
            'obstacle_name'
        ).value

        self.model_file = self.get_parameter(
            'model_file'
        ).value

        self.x = self.get_parameter(
            'start_x'
        ).value

        self.y = self.get_parameter(
            'start_y'
        ).value

        self.z = self.get_parameter(
            'start_z'
        ).value

        self.min_x = self.get_parameter(
            'min_x'
        ).value

        self.max_x = self.get_parameter(
            'max_x'
        ).value

        self.speed = self.get_parameter(
            'speed'
        ).value

        self.update_rate = self.get_parameter(
            'update_rate'
        ).value

        self.auto_spawn = self.get_parameter(
            'auto_spawn'
        ).value

        # =========================
        # Internal state
        # =========================

        self.direction = 1.0

        self.spawned = False

        # =========================
        # Locate SDF
        # =========================

        package_share = get_package_share_directory(
            'dynamic_obstacle'
        )

        self.sdf_path = os.path.join(
            package_share,
            'models',
            self.model_file
        )

        self.get_logger().info(
            f'SDF file: {self.sdf_path}'
        )

        # =========================
        # Spawn obstacle
        # =========================

        if self.auto_spawn:

            self.spawn_obstacle()

            # 给 Gazebo 一点时间完成实体创建
            time.sleep(0.5)

        # =========================
        # Timer
        # =========================

        period = 1.0 / self.update_rate

        self.timer = self.create_timer(
            period,
            self.update_obstacle
        )

        self.get_logger().info(
            'Dynamic obstacle manager started.'
        )

        self.get_logger().info(
            f'World: {self.world_name}'
        )

        self.get_logger().info(
            f'Obstacle: {self.obstacle_name}'
        )

        self.get_logger().info(
            f'Range: [{self.min_x}, {self.max_x}] m'
        )

        self.get_logger().info(
            f'Speed: {self.speed} m/s'
        )

    # ==========================================================
    # Spawn obstacle
    # ==========================================================

    def spawn_obstacle(self):

        if not os.path.exists(self.sdf_path):

            self.get_logger().error(
                f'SDF file does not exist: {self.sdf_path}'
            )

            return False

        command = [
            'ign',
            'service',
            '-s',
            f'/world/{self.world_name}/create',

            '--reqtype',
            'ignition.msgs.EntityFactory',

            '--reptype',
            'ignition.msgs.Boolean',

            '--timeout',
            '1000',

            '--req',

            (
                f'sdf_filename: "{self.sdf_path}", '
                f'name: "{self.obstacle_name}", '
                f'pose: {{'
                f'position: {{'
                f'x: {self.x}, '
                f'y: {self.y}, '
                f'z: {self.z}'
                f'}}'
                f'}}'
            )
        ]

        self.get_logger().info(
            f'Spawning obstacle at '
            f'({self.x:.2f}, {self.y:.2f}, {self.z:.2f})'
        )

        try:

            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0
            )

            if result.returncode != 0:

                self.get_logger().error(
                    'Failed to spawn obstacle.'
                )

                self.get_logger().error(
                    result.stderr
                )

                return False

            if 'data: true' in result.stdout:

                self.spawned = True

                self.get_logger().info(
                    'Obstacle spawned successfully.'
                )

                return True

            else:

                self.get_logger().error(
                    f'Gazebo create response: '
                    f'{result.stdout}'
                )

                return False

        except subprocess.TimeoutExpired:

            self.get_logger().error(
                'Gazebo spawn service timed out.'
            )

            return False

    # ==========================================================
    # Set obstacle pose
    # ==========================================================

    def set_pose(self, x):

        command = [
            'ign',
            'service',
            '-s',
            f'/world/{self.world_name}/set_pose',

            '--reqtype',
            'ignition.msgs.Pose',

            '--reptype',
            'ignition.msgs.Boolean',

            '--timeout',
            '1000',

            '--req',

            (
                f'name: "{self.obstacle_name}", '
                f'position: {{'
                f'x: {x}, '
                f'y: {self.y}, '
                f'z: {self.z}'
                f'}}'
            )
        ]

        try:

            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0
            )

            if result.returncode != 0:

                self.get_logger().error(
                    'Failed to set obstacle pose.'
                )

                return False

            return 'data: true' in result.stdout

        except subprocess.TimeoutExpired:

            self.get_logger().error(
                'Gazebo set_pose service timed out.'
            )

            return False

    # ==========================================================
    # Update obstacle
    # ==========================================================

    def update_obstacle(self):

        if not self.spawned:
            return

        dt = 1.0 / self.update_rate

        self.x += self.direction * self.speed * dt

        # 到达右边界
        if self.x >= self.max_x:

            self.x = self.max_x
            self.direction = -1.0

        # 到达左边界
        elif self.x <= self.min_x:

            self.x = self.min_x
            self.direction = 1.0

        success = self.set_pose(self.x)

        if not success:

            self.get_logger().warning(
                'Failed to update obstacle pose.'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleManager()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
