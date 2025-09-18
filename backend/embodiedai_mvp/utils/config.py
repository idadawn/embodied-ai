"""YAML-backed configuration loader for the EmbodiedAI MVP."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field


class RerunConfig(BaseModel):
    enabled: bool = True
    web_port: int = Field(default=9876, ge=1024, le=65535)


class SimulationConfig(BaseModel):
    mujoco_model_path: Path
    control_rate_hz: float = Field(default=50.0, gt=0)
    desk_half_extents: tuple[float, float, float] = (0.5, 0.5, 0.1)
    desk_center: tuple[float, float, float] = (0.0, 0.0, 0.1)


class MCPConfig(BaseModel):
    host: str = '0.0.0.0'
    port: int = Field(default=8080, ge=1, le=65535)
    allowed_origins: list[str] = Field(default_factory=lambda: ['*'])


class AppConfig(BaseModel):
    simulation: SimulationConfig
    rerun: RerunConfig = RerunConfig()
    mcp: MCPConfig = MCPConfig()


def load_config(path: str | Path) -> AppConfig:
    data = _read_yaml(path)
    return AppConfig(**data)


def _read_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).expanduser().open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}
