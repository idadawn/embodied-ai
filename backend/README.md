# EmbodiedAI MVP Backend

This package contains the ROS 2 simulation nodes, the MCP HTTP server, and supporting utilities for the EmbodiedAI MVP.

## Installation

### Using uv (recommended)

```bash
uv sync
```

### Using pip

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e .[dev]
```

## Launching the stack

```bash
source /opt/ros/humble/setup.bash
ros2 launch embodiedai_mvp sim.launch.py
```

The launch file starts:
- `simulation_node` – MuJoCo-driven scene publisher with Rerun logging
- `vision_node` – stubbed pose estimation service
- `grasp_action_server` – simulated gripper action server
- `mcp_server` – MCP bridge exposing REST endpoints for the frontend

## Standalone processes

Each executable supports `--config /path/to/app.yaml` to override the default configuration file at `config/app.yaml`.

## Tests

```bash
uv run pytest
```

The test suite currently validates that the fallback `BrainClient` yields a grasp plan.

