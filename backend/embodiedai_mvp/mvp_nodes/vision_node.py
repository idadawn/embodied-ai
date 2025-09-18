"""Vision helper node that exposes a simple pose estimation service."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
try:
    import rclpy  # type: ignore
    from example_interfaces.srv import Trigger  # type: ignore
    from geometry_msgs.msg import PoseStamped  # type: ignore
    from rclpy.node import Node  # type: ignore
    _RCLPY_AVAILABLE = True
except Exception:  # noqa: BLE001
    rclpy = None  # type: ignore[assignment]
    Trigger = object  # type: ignore[assignment]
    PoseStamped = object  # type: ignore[assignment]
    Node = object  # type: ignore[assignment]
    _RCLPY_AVAILABLE = False

from embodiedai_mvp.utils import rerun_utils
from embodiedai_mvp.utils.config import AppConfig, load_config

TARGET_FRAME = 'world'
TARGET_OBJECT = 'coke_can'


@dataclass(slots=True)
class PoseEstimate:
    position: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]


class VisionNode(Node):
    """Simulated vision stack that returns a fixed pose estimate for the coke can."""

    def __init__(self, config: AppConfig | None = None):
        super().__init__('vision_node')
        self._config = config or self._config_from_parameters()

        if self._config.rerun.enabled:
            rerun_utils.setup_rerun(web_port=None)

        if _RCLPY_AVAILABLE:
            self._service = self.create_service(Trigger, '/vision/estimate_coke_pose', self._on_trigger)
            self._pose_pub = self.create_publisher(PoseStamped, '/vision/coke_pose', 10)
        else:
            self._service = None
            self._pose_pub = None

        self.get_logger().info('Vision node ready to estimate %s pose', TARGET_OBJECT)

    def _config_from_parameters(self) -> AppConfig:
        default = Path(__file__).resolve().parents[2] / 'config' / 'app.yaml'
        declared = self.declare_parameter('config_path', str(default))
        return load_config(declared.value)

    def _on_trigger(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        estimate = self._fake_estimate()
        if not self._pose_pub:
            return response
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = TARGET_FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = estimate.position
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = estimate.quaternion
        self._pose_pub.publish(pose)

        if self._config.rerun.enabled:
            rerun_utils.log_objects({TARGET_OBJECT: estimate.position})

        response.success = True
        response.message = (
            f"Pose estimate: position={estimate.position}, quaternion={estimate.quaternion}"
        )
        return response

    def _fake_estimate(self) -> PoseEstimate:
        position = (0.2 + float(np.random.normal(scale=0.01)), float(np.random.normal(scale=0.01)), 0.15)
        quaternion = (0.0, 0.0, 0.0, 1.0)
        return PoseEstimate(position=position, quaternion=quaternion)


def _parse_args(cli_args: list[str]) -> tuple[str | None, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--config', dest='config_path')
    parsed, remaining = parser.parse_known_args(cli_args)
    return parsed.config_path, remaining


def main(args: list[str] | None = None) -> None:
    cli_args = list(args) if args is not None else sys.argv[1:]
    config_path, ros_args = _parse_args(cli_args)
    if not _RCLPY_AVAILABLE:
        raise SystemExit('ROS 2 (rclpy) 未安装，无法运行视觉节点。请参阅 README 获取安装说明或改用 Docker。')
    rclpy.init(args=ros_args)  # type: ignore[call-arg]
    config = load_config(config_path) if config_path else None
    node = VisionNode(config=config)
    try:
        rclpy.spin(node)  # type: ignore[arg-type]
    finally:
        node.destroy_node()
        rclpy.shutdown()  # type: ignore[call-arg]


if __name__ == '__main__':
    main()
