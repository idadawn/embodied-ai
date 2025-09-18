"""Action server that coordinates a simple grasp routine."""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
try:
    import rclpy  # type: ignore
    from control_msgs.action import GripperCommand  # type: ignore
    from rclpy.action import ActionServer, CancelResponse, GoalResponse, ServerGoalHandle  # type: ignore
    from rclpy.node import Node  # type: ignore
    _RCLPY_AVAILABLE = True
except Exception:  # noqa: BLE001
    rclpy = None  # type: ignore[assignment]
    GripperCommand = object  # type: ignore[assignment]
    ActionServer = object  # type: ignore[assignment]
    CancelResponse = object  # type: ignore[assignment]
    GoalResponse = object  # type: ignore[assignment]
    class ServerGoalHandle:  # type: ignore[no-redef]
        request: object
        def publish_feedback(self, *_: object, **__: object) -> None: ...
        def succeed(self) -> None: ...
    Node = object  # type: ignore[assignment]
    _RCLPY_AVAILABLE = False

from embodiedai_mvp.utils import rerun_utils
from embodiedai_mvp.utils.config import AppConfig, load_config


class GraspActionServer(Node):
    """Simulated gripper action that closes fingers around the coke can."""

    def __init__(self, config: AppConfig | None = None):
        super().__init__('grasp_action_server')
        self._config = config or self._config_from_parameters()

        if self._config.rerun.enabled:
            rerun_utils.setup_rerun(web_port=None)

        if _RCLPY_AVAILABLE:
            self._action_server = ActionServer(
                self,
                GripperCommand,
                '/grasp/execute',
                execute_callback=self._execute_cb,
                goal_callback=self._goal_cb,
                cancel_callback=self._cancel_cb,
            )

        self.get_logger().info('Grasp action server available on /grasp/execute')

    def _config_from_parameters(self) -> AppConfig:
        default = Path(__file__).resolve().parents[2] / 'config' / 'app.yaml'
        declared = self.declare_parameter('config_path', str(default))
        return load_config(declared.value)

    def _goal_cb(self, _goal_request: GripperCommand.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.REJECT

    async def _execute_cb(self, goal_handle: ServerGoalHandle) -> GripperCommand.Result:
        goal = goal_handle.request
        self.get_logger().info('Executing grasp towards effort %.2f', goal.command.max_effort)

        feedback = GripperCommand.Feedback()
        for position in np.linspace(0.08, goal.command.position, num=10):
            feedback.position = float(position)
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.05)

        goal_handle.succeed()

        if self._config.rerun.enabled:
            trajectory = self._simulate_closing_arc()
            rerun_utils.log_end_effector_trajectory(trajectory)

        result = GripperCommand.Result()
        result.position = goal.command.position
        result.effort = goal.command.max_effort
        result.stalled = False
        result.reached_goal = True
        return result

    def _simulate_closing_arc(self) -> Iterable[np.ndarray]:
        radius = 0.05
        angles = np.linspace(0.0, math.pi / 3, num=20)
        for angle in angles:
            yield np.array([0.2 - radius * (1 - math.cos(angle)), radius * math.sin(angle), 0.12])


def _parse_args(cli_args: list[str]) -> tuple[str | None, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--config', dest='config_path')
    parsed, remaining = parser.parse_known_args(cli_args)
    return parsed.config_path, remaining


def main(args: list[str] | None = None) -> None:
    cli_args = list(args) if args is not None else sys.argv[1:]
    config_path, ros_args = _parse_args(cli_args)
    if not _RCLPY_AVAILABLE:
        raise SystemExit('ROS 2 (rclpy) 未安装，无法运行抓取 action 服务器。请参阅 README 获取安装说明或改用 Docker。')
    rclpy.init(args=ros_args)  # type: ignore[call-arg]
    config = load_config(config_path) if config_path else None
    node = GraspActionServer(config=config)
    try:
        rclpy.spin(node)  # type: ignore[arg-type]
    finally:
        node.destroy_node()
        rclpy.shutdown()  # type: ignore[call-arg]


if __name__ == '__main__':
    main()
