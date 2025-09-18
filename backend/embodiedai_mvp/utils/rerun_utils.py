"""Helpers that centralise Rerun logging setup for EmbodiedAI nodes."""

from __future__ import annotations

from typing import Iterable, Mapping, Sequence

import numpy as np
import rerun as rr

DEFAULT_RECORDING = "embodiedai_mvp"


def setup_rerun(*, recording: str = DEFAULT_RECORDING, web_port: int | None = 9876) -> None:
    """Configure a shared Rerun recording and optionally spawn ``rerun-serve``."""
    rr.init(recording, spawn=False)
    if web_port is not None:
        try:
            rr.serve(web_port=web_port)
        except RuntimeError:
            # The server might already be running in another node; ignore the clash.
            pass


def log_static_scene(*, desk_half_extents: Sequence[float], desk_center: Sequence[float]) -> None:
    """Log static reference geometry only once at boot."""
    rr.log(
        "/world/desk",
        rr.Boxes3D(half_sizes=[desk_half_extents], centers=[desk_center], colors=[[90, 110, 150]]),
    )


def log_objects(objects: Mapping[str, Sequence[float]]) -> None:
    """Log simple box representations for props on the desk."""
    for name, position in objects.items():
        rr.log(
            f"/world/props/{name}",
            rr.Boxes3D(half_sizes=[[0.03, 0.03, 0.06]], centers=[position], colors=[[220, 80, 80]]),
        )


def log_robot_links(transforms: Mapping[str, np.ndarray]) -> None:
    """Log transform matrices for a robot kinematic chain in world coordinates."""
    for link, matrix in transforms.items():
        translation = matrix[:3, 3]
        rotation = matrix[:3, :3]
        rr.log(
            f"/world/robot/{link}",
            rr.Transform3D(translation=translation.tolist(), rotation=rr.Rotation3D(rotation, form="matrix")),
        )


def log_end_effector_trajectory(points: Iterable[Sequence[float]]) -> None:
    """Visualise the executed tool path as a 3D line strip."""
    positions = np.array(list(points))
    if positions.size == 0:
        return
    rr.log("/world/robot/end_effector/path", rr.LineStrips3D(positions=[positions.tolist()]))
