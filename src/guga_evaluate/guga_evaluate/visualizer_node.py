"""Live Matplotlib dashboard for guga navigation evaluation metrics."""

from __future__ import annotations

import json
import math
import signal
from collections import deque
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def nested_number(value: dict, group: str, key: str) -> float:
    item = value.get(group, {}).get(key)
    if isinstance(item, (int, float)) and math.isfinite(item):
        return float(item)
    return math.nan


def format_number(value, unit: str, digits: int = 3) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return "--"
    return f"{value:.{digits}f} {unit}".rstrip()


class MetricsReceiver(Node):
    """Receive evaluator snapshots and retain a bounded time window."""

    def __init__(self) -> None:
        super().__init__("guga_evaluate_visualizer")
        self.declare_parameter("metrics_topic", "evaluation_metrics")
        self.declare_parameter("history_sec", 30.0)
        self.declare_parameter("update_period_sec", 0.1)
        self.metrics_topic = self.get_parameter("metrics_topic").value
        self.history_sec = float(self.get_parameter("history_sec").value)
        self.update_period_sec = float(
            self.get_parameter("update_period_sec").value)
        self.samples: deque[dict] = deque()
        self.latest: Optional[dict] = None
        self.start_time: Optional[float] = None
        self.create_subscription(String, self.metrics_topic, self._on_metrics, 10)

    def _on_metrics(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            stamp = float(value["time_sec"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"Invalid evaluation metrics: {error}")
            return
        if self.start_time is None:
            self.start_time = stamp
        value["plot_time_sec"] = stamp - self.start_time
        self.samples.append(value)
        self.latest = value
        cutoff = value["plot_time_sec"] - self.history_sec
        while self.samples and self.samples[0]["plot_time_sec"] < cutoff:
            self.samples.popleft()


class Dashboard:
    """Render live command, response, tracking, and health plots."""

    def __init__(self, receiver: MetricsReceiver, executor, plt) -> None:
        self.receiver = receiver
        self.executor = executor
        self.plt = plt
        self.figure, axes = plt.subplots(2, 2, figsize=(12, 7))
        self.figure.canvas.manager.set_window_title("Guga Evaluate")
        self.figure.suptitle("Guga Navigation Evaluation", fontsize=15)
        self.speed_axis = axes[0][0]
        self.angular_axis = axes[0][1]
        self.error_axis = axes[1][0]
        self.status_axis = axes[1][1]

        self.command_speed, = self.speed_axis.plot(
            [], [], label="command", linewidth=1.8)
        self.actual_speed, = self.speed_axis.plot(
            [], [], label="actual", linewidth=1.8)
        self.command_angular, = self.angular_axis.plot(
            [], [], label="command", linewidth=1.8)
        self.actual_angular, = self.angular_axis.plot(
            [], [], label="actual", linewidth=1.8)
        self.cross_track, = self.error_axis.plot(
            [], [], label="cross-track", linewidth=1.8)
        self.goal_distance, = self.error_axis.plot(
            [], [], label="goal distance", linewidth=1.8)

        self._configure_axis(
            self.speed_axis, "Linear speed", "Speed (m/s)")
        self._configure_axis(
            self.angular_axis, "Angular speed", "Rate (rad/s)")
        self._configure_axis(
            self.error_axis, "Tracking error", "Distance (m)")
        self.status_axis.set_title("Current status")
        self.status_axis.axis("off")
        self.status_text = self.status_axis.text(
            0.03, 0.95, "Waiting for evaluation metrics...",
            transform=self.status_axis.transAxes, va="top", ha="left",
            family="monospace", fontsize=11, linespacing=1.5)
        self.figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

    @staticmethod
    def _configure_axis(axis, title: str, ylabel: str) -> None:
        axis.set_title(title)
        axis.set_xlabel("Time (s)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper left")

    def _update_lines(self) -> None:
        samples = list(self.receiver.samples)
        if not samples:
            return
        times = [sample["plot_time_sec"] for sample in samples]
        self.command_speed.set_data(times, [
            nested_number(sample, "command", "linear_speed_mps")
            for sample in samples])
        self.actual_speed.set_data(times, [
            nested_number(sample, "actual", "linear_speed_mps")
            for sample in samples])
        self.command_angular.set_data(times, [
            nested_number(sample, "command", "wz_radps")
            for sample in samples])
        self.actual_angular.set_data(times, [
            nested_number(sample, "actual", "wz_radps")
            for sample in samples])
        self.cross_track.set_data(times, [
            nested_number(sample, "tracking", "cross_track_error_m")
            for sample in samples])
        self.goal_distance.set_data(times, [
            nested_number(sample, "tracking", "goal_distance_m")
            for sample in samples])

        latest_time = times[-1]
        left = max(0.0, latest_time - self.receiver.history_sec)
        right = max(self.receiver.history_sec, latest_time)
        for axis in (
            self.speed_axis, self.angular_axis, self.error_axis,
        ):
            axis.set_xlim(left, right)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)

    def _update_status(self) -> None:
        latest = self.receiver.latest
        if latest is None:
            return
        tracking = latest.get("tracking", {})
        aggregate = latest.get("aggregate", {})
        actual = latest.get("actual", {})
        reached = aggregate.get("goal_reached")
        if reached is True:
            goal_state = "REACHED"
        elif aggregate.get("goal_id") is not None:
            goal_state = "TRACKING"
        else:
            goal_state = "NO GOAL"
        recording = "ON" if latest.get("save_data") else "OFF"
        self.status_text.set_text(
            f"mode              {latest.get('mode', '--')}\n"
            f"recording         {recording}\n"
            f"velocity source   {actual.get('source', '--')}\n"
            f"goal              {goal_state}\n"
            f"\n"
            f"speed error       "
            f"{format_number(tracking.get('speed_error_mps'), 'm/s')}\n"
            f"speed RMSE        "
            f"{format_number(aggregate.get('speed_error_rmse_mps'), 'm/s')}\n"
            f"cross-track RMSE  "
            f"{format_number(aggregate.get('cross_track_rmse_m'), 'm')}\n"
            f"heading RMSE      "
            f"{format_number(aggregate.get('heading_error_rmse_rad'), 'rad')}\n"
            f"goal distance     "
            f"{format_number(tracking.get('goal_distance_m'), 'm')}\n"
            f"\n"
            f"cmd / odom rate   "
            f"{format_number(aggregate.get('command_hz'), 'Hz', 1)} / "
            f"{format_number(aggregate.get('odometry_hz'), 'Hz', 1)}"
        )

    def update(self, _frame):
        if not rclpy.ok():
            self.plt.close(self.figure)
            return ()
        for _ in range(10):
            self.executor.spin_once(timeout_sec=0.0)
        self._update_lines()
        self._update_status()
        return (
            self.command_speed, self.actual_speed,
            self.command_angular, self.actual_angular,
            self.cross_track, self.goal_distance, self.status_text,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    receiver = MetricsReceiver()
    executor = SingleThreadedExecutor()
    executor.add_node(receiver)
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        if plt.get_backend().lower() == "agg":
            receiver.get_logger().error(
                "No interactive display is available; disable visualization "
                "or run from a graphical desktop session.")
            return
        dashboard = Dashboard(receiver, executor, plt)
        animation = FuncAnimation(
            dashboard.figure, dashboard.update,
            interval=max(20, int(receiver.update_period_sec * 1000)),
            cache_frame_data=False)
        dashboard.figure._guga_animation = animation

        def close_dashboard(_signum, _frame) -> None:
            plt.close(dashboard.figure)

        signal.signal(signal.SIGINT, close_dashboard)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(receiver)
        receiver.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
