# EmbodiedAI MVP

EmbodiedAI MVP is an end-to-end prototype that turns natural language into grasping behaviour inside a MuJoCo simulation while streaming a rich 3D digital twin to the browser through Rerun Web Viewer. The repository bundles a Next.js control center and a ROS 2 + MCP backend.

## Repository layout

- `frontend/` – Next.js 15 dashboard, Zustand store, and embedded Rerun viewer.
- `backend/` – ROS 2 inspired Python package with MuJoCo simulation nodes, MCP server, and Rerun utilities.
- `pnpm-workspace.yaml` – pnpm workspace definition so the root can install the frontend with `pnpm install`.

## Quick start

### Frontend (pnpm)

```bash
cd frontend
pnpm install
pnpm dev
```

The dashboard is served on `http://localhost:3000/dashboard` and expects the MCP API at `http://localhost:8080/api` plus a Rerun WebSocket on `ws://localhost:9876` (configurable via `.env.local`).

### Backend（uv 或 virtualenv）

```bash
cd backend
uv sync  # 或: python3 -m venv .venv && source .venv/bin/activate && pip install -e .
# MCP HTTP 服务（无 ROS 时也可运行，会使用内置降级桥接）：
uv run embodiedai-mcp

# 需要 ROS 2 的节点（仅在安装了 ROS 2/rclpy 后可运行）：
ros2 launch embodiedai_mvp sim.launch.py
```

The launch file starts the MuJoCo simulation node, the vision stub, the grasp action server, and the MCP HTTP bridge in one shot. The MCP endpoint listens on `http://0.0.0.0:8080` by default and exposes `POST /api/commands` and `GET /api/health`.

## Environment variables

Copy `.env.example` next to the frontend/backend entry points to tweak ports and endpoints. By default the frontend looks for:

```
NEXT_PUBLIC_RERUN_WS_URL=ws://localhost:9876
NEXT_PUBLIC_MCP_API_URL=http://localhost:8080/api
```

## Testing

Backend 单测依赖 `pytest`：

```bash
cd backend
uv run pytest
```

(If `uv` is unavailable, activate the virtualenv you used for `pip install -e .` and call `pytest` directly.)

## Notes

- `backend/urdf/mjcf/embodiedai_hand.xml` 中的 MuJoCo 模型是占位符，可按需替换为真实场景。
- MCP 服务在未安装 ROS 2（rclpy）时会自动降级为本地假桥接，只做参数校验与回传；需要真实抓取链路时请安装 ROS 2 并运行抓取 action 服务器。
- Rerun 日志工具支持多发布者；主模拟节点负责拉起 `rerun-serve` 进程。

### macOS 指南（无 ROS 情况下运行 MCP 服务）
- 直接运行 MCP 服务（无需安装 ROS）：
  ```bash
  cd backend
  uv sync
  uv run embodiedai-mcp
  ```
  前端默认访问 `http://localhost:8080/api`。

### 在本机启用 ROS 2（可选）
- 推荐使用 Docker：
  - 准备一个带 `ros:humble`（或 `iron`）基础镜像，安装 `python3-colcon-common-extensions` 与 `rclpy`，将本仓库 `backend` 目录作为 volume 挂载运行。
  - 在容器内执行：
    ```bash
    ros2 launch embodiedai_mvp sim.launch.py
    ```
- 或使用 Linux/WSL2 原生安装 ROS 2，再在宿主/WSL 内运行上述命令。

注意：macOS 上原生安装 `rclpy` 难度较高，建议走 Docker 或 WSL2（在 Apple Silicon 上可使用 UTM/虚拟机的 Ubuntu）。

