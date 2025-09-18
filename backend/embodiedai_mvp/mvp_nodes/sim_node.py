"""ROS 2 node that maintains the MuJoCo simulation and streams state to Rerun."""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path
from typing import Deque

import mujoco
import numpy as np
try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from sensor_msgs.msg import JointState  # type: ignore
    _RCLPY_AVAILABLE = True
except Exception:  # noqa: BLE001
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment]
    class JointState:  # minimal stub for type use
        def __init__(self) -> None:
            self.header = type('H', (), {'stamp': None})()
            self.name = []
            self.position = []
            self.velocity = []
    _RCLPY_AVAILABLE = False

from embodiedai_mvp.utils import rerun_utils
from embodiedai_mvp.utils.config import AppConfig, load_config

MAX_TRAJECTORY_POINTS = 1024


class SimulationNode(Node):
    """Maintain a MuJoCo scene, publish joint state, and log Rerun frames."""

    def __init__(self, config: AppConfig | None = None):
        super().__init__('simulation_node')
        self._config = config or self._config_from_parameters()
        self._model = mujoco.MjModel.from_xml_path(str(self._config.simulation.mujoco_model_path))
        self._data = mujoco.MjData(self._model)
        self._dt = 1.0 / self._config.simulation.control_rate_hz
        self._last_step_ts = time.perf_counter()
        self._trajectory: Deque[np.ndarray] = deque(maxlen=MAX_TRAJECTORY_POINTS)

        if self._config.rerun.enabled:
            rerun_utils.setup_rerun(web_port=self._config.rerun.web_port)
            rerun_utils.log_static_scene(
                desk_half_extents=self._config.simulation.desk_half_extents,
                desk_center=self._config.simulation.desk_center,
            )
            rerun_utils.log_objects({
                'coke_can': (0.2, 0.0, 0.16),
                'cup': (-0.18, -0.05, 0.14)
            })

        if _RCLPY_AVAILABLE:
            self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)
            self._timer = self.create_timer(self._dt, self._on_timer)
        else:
            self._joint_pub = None

        self.get_logger().info('Simulation node initialised with control dt %.4fs', self._dt)

    def _config_from_parameters(self) -> AppConfig:
        default = Path(__file__).resolve().parents[2] / 'config' / 'app.yaml'
        declared = self.declare_parameter('config_path', str(default))
        return load_config(declared.value)

    def _on_timer(self) -> None:
        now = time.perf_counter()
        delta = now - self._last_step_ts
        self._last_step_ts = now
        steps = max(1, int(delta / self._dt))
        for _ in range(steps):
            mujoco.mj_step(self._model, self._data)

        self._publish_joint_state()
        self._record_rerun()

    def _publish_joint_state(self) -> None:
        if not self._joint_pub:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        joint_names = []
        for joint_id in range(self._model.njnt):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            joint_names.append(name or f'joint_{joint_id}')
        msg.name = joint_names
        msg.position = self._data.qpos[: self._model.njnt].tolist()
        msg.velocity = self._data.qvel[: self._model.njnt].tolist()
        self._joint_pub.publish(msg)

    def _record_rerun(self) -> None:
        if not self._config.rerun.enabled:
            return

        transforms = self._compute_forward_kinematics()
        if transforms:
            rerun_utils.log_robot_links(transforms)
            end_effector_name = next(
                (name for name in ('gripper', 'hand', 'tool0', 'wrist', 'end_effector') if name in transforms),
                next(iter(transforms))
            )
            ee_pos = transforms[end_effector_name][:3, 3]
            self._trajectory.append(ee_pos.copy())
            rerun_utils.log_end_effector_trajectory(self._trajectory)

    def _compute_forward_kinematics(self) -> dict[str, np.ndarray]:
        mujoco.mj_forward(self._model, self._data)
        transforms: dict[str, np.ndarray] = {}
        for body_id in range(self._model.nbody):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if not name:
                continue
            matrix = np.eye(4)
            matrix[:3, :3] = self._data.xmat[body_id].reshape(3, 3)
            matrix[:3, 3] = self._data.xpos[body_id]
            transforms[name] = matrix
        return transforms


def _parse_args(cli_args: list[str]) -> tuple[str | None, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--config', dest='config_path')
    parsed, remaining = parser.parse_known_args(cli_args)
    return parsed.config_path, remaining


def main(args: list[str] | None = None) -> None:
    cli_args = list(args) if args is not None else sys.argv[1:]
    config_path, ros_args = _parse_args(cli_args)
    if not _RCLPY_AVAILABLE:
        raise SystemExit('ROS 2 (rclpy) 未安装，无法运行模拟节点。请参阅 README 获取安装说明或改用 Docker。')
    rclpy.init(args=ros_args)  # type: ignore[call-arg]
    config = load_config(config_path) if config_path else None
    node = SimulationNode(config=config)
    try:
        rclpy.spin(node)  # type: ignore[arg-type]
    finally:
        node.destroy_node()
        rclpy.shutdown()  # type: ignore[call-arg]


if __name__ == '__main__':
    main()
