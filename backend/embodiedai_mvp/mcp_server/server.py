"""HTTP bridge that exposes the MCP command surface to the frontend."""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from dataclasses import asdict, dataclass
from functools import partial
from pathlib import Path
from typing import Awaitable, Callable

from aiohttp import web

try:
    import rclpy  # type: ignore
    from rclpy.action import ActionClient  # type: ignore
    from rclpy.executors import SingleThreadedExecutor  # type: ignore
    from control_msgs.action import GripperCommand  # type: ignore
    _RCLPY_AVAILABLE = True
except Exception:  # noqa: BLE001
    rclpy = None  # type: ignore[assignment]
    ActionClient = None  # type: ignore[assignment]
    SingleThreadedExecutor = None  # type: ignore[assignment]
    GripperCommand = None  # type: ignore[assignment]
    _RCLPY_AVAILABLE = False

from embodiedai_mvp.brain.brain_client import BrainClient
from embodiedai_mvp.utils.config import AppConfig, load_config

logger = logging.getLogger(__name__)


@dataclass
class CommandResult:
    command: str
    status: str
    reasoning: str
    confidence: float
    effort: float
    gripper_width: float


class SimpleGraspResult:
    effort: float
    position: float

    def __init__(self, *, effort: float, position: float) -> None:
        self.effort = float(effort)
        self.position = float(position)


if _RCLPY_AVAILABLE:
    class ActionBridge:
        """Helper that proxies grasp goals to the ROS action server synchronously."""

        def __init__(self) -> None:
            self._node = rclpy.create_node('mcp_action_bridge')  # type: ignore[call-arg]
            self._executor = SingleThreadedExecutor()  # type: ignore[call-arg]
            self._executor.add_node(self._node)
            self._action_client = ActionClient(self._node, GripperCommand, '/grasp/execute')  # type: ignore[call-arg]

        def shutdown(self) -> None:
            self._executor.remove_node(self._node)
            self._node.destroy_node()

        def execute_grasp(self, *, position: float, effort: float) -> SimpleGraspResult:
            if not self._action_client.wait_for_server(timeout_sec=2.0):
                raise RuntimeError('Grasp action服务器未就绪。')

            goal_msg = GripperCommand.Goal()  # type: ignore[call-arg]
            goal_msg.command.position = float(position)
            goal_msg.command.max_effort = float(effort)

            send_future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self._node, send_future, executor=self._executor)  # type: ignore[arg-type]
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError('Grasp action目标被拒绝。')

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future, executor=self._executor)  # type: ignore[arg-type]
            result = result_future.result().result
            return SimpleGraspResult(effort=result.effort, position=result.position)  # type: ignore[attr-defined]
else:
    class ActionBridge:
        """Fallback bridge used when ROS 2 (rclpy) 不可用。"""

        def __init__(self) -> None:  # noqa: D401
            self._ready = True

        def shutdown(self) -> None:
            return None

        def execute_grasp(self, *, position: float, effort: float) -> SimpleGraspResult:
            clamped_pos = max(0.0, min(float(position), 0.1))
            clamped_effort = max(0.0, float(effort))
            return SimpleGraspResult(effort=clamped_effort, position=clamped_pos)


class MCPServer:
    def __init__(self, config: AppConfig):
        self._config = config
        self._app = web.Application(middlewares=[self._cors_middleware])
        self._app.router.add_post('/api/commands', self._handle_command)
        self._app.router.add_get('/api/health', self._handle_health)
        self._runner: web.AppRunner | None = None
        self._site: web.TCPSite | None = None
        self._action_bridge = ActionBridge()

    @web.middleware
    async def _cors_middleware(self, request: web.Request, handler: Callable[[web.Request], Awaitable[web.StreamResponse]]) -> web.StreamResponse:
        if request.method == 'OPTIONS':
            response = web.Response(status=204)
        else:
            response = await handler(request)
        self._apply_cors(request, response)
        return response

    def _apply_cors(self, request: web.Request, response: web.StreamResponse) -> None:
        origins = self._config.mcp.allowed_origins
        origin_header = request.headers.get('Origin')
        if '*' in origins:
            response.headers['Access-Control-Allow-Origin'] = '*'
        elif origin_header and origin_header in origins:
            response.headers['Access-Control-Allow-Origin'] = origin_header
        elif origins:
            response.headers['Access-Control-Allow-Origin'] = origins[0]
        response.headers['Access-Control-Allow-Methods'] = 'POST, GET, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'

    async def start(self) -> None:
        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        host = self._config.mcp.host
        port = self._config.mcp.port
        self._site = web.TCPSite(self._runner, host=host, port=port)
        await self._site.start()
        logger.info('MCP server listening at http://%s:%s', host, port)

    async def stop(self) -> None:
        if self._site:
            await self._site.stop()
            self._site = None
        if self._runner:
            await self._runner.cleanup()
            self._runner = None
        self._action_bridge.shutdown()

    async def _handle_health(self, _request: web.Request) -> web.Response:
        return web.json_response({'status': 'ok'})

    async def _handle_command(self, request: web.Request) -> web.Response:
        payload = await request.json()
        command = str(payload.get('command', '')).strip()
        if not command:
            return web.json_response({'error': 'command is required'}, status=400)

        async with BrainClient(endpoint=None) as brain:
            plan = await brain.plan_grasp(command)

        loop = asyncio.get_running_loop()
        try:
            result = await loop.run_in_executor(
                None,
                partial(self._action_bridge.execute_grasp, position=plan.gripper_width, effort=plan.grasp_effort)
            )
        except Exception as exc:  # noqa: BLE001
            logger.exception('Failed to execute grasp goal')
            return web.json_response({'error': str(exc)}, status=500)

        command_result = CommandResult(
            command=command,
            status='succeeded',
            reasoning=plan.reasoning,
            confidence=plan.confidence,
            effort=float(getattr(result, 'effort')),
            gripper_width=float(getattr(result, 'position')),
        )
        return web.json_response(asdict(command_result))


async def _run_server(config: AppConfig) -> None:
    server = MCPServer(config)
    await server.start()

    stop_event = asyncio.Event()

    def _shutdown_handler() -> None:
        stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _shutdown_handler)
        except NotImplementedError:
            pass

    await stop_event.wait()
    await server.stop()


def _load_app_config(config_path: str | None) -> AppConfig:
    base = Path(__file__).resolve().parents[2]
    path = Path(config_path).expanduser() if config_path else base / 'config' / 'app.yaml'
    return load_config(path)


def _parse_args(cli_args: list[str]) -> tuple[str | None, list[str]]:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--config', dest='config_path')
    parsed, remaining = parser.parse_known_args(cli_args)
    return parsed.config_path, remaining


def main(config_path: str | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s %(name)s: %(message)s')
    cli_args = sys.argv[1:] if config_path is None else []
    override, ros_args = _parse_args(cli_args)
    config_override = config_path or override
    if _RCLPY_AVAILABLE:
        rclpy.init(args=ros_args)  # type: ignore[call-arg]
    try:
        config = _load_app_config(config_override)
        asyncio.run(_run_server(config))
    finally:
        if _RCLPY_AVAILABLE:
            rclpy.shutdown()  # type: ignore[call-arg]


if __name__ == '__main__':
    main()
