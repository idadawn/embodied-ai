"""Thin wrapper around the VLA brain or MCP agent."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Dict

import aiohttp


@dataclass(slots=True)
class BrainPlan:
    """Representation of the MCP brain response for a grasp command."""

    command: str
    confidence: float
    gripper_width: float
    grasp_effort: float
    reasoning: str


class BrainClient:
    """Client that talks to the VLA/MCP orchestrator to turn language into actions."""

    def __init__(self, endpoint: str | None = None) -> None:
        self._endpoint = endpoint
        self._session: aiohttp.ClientSession | None = None

    async def __aenter__(self) -> "BrainClient":
        await self._ensure_session()
        return self

    async def __aexit__(self, *exc_info: Any) -> None:  # type: ignore[override]
        if self._session:
            await self._session.close()
            self._session = None

    async def _ensure_session(self) -> aiohttp.ClientSession:
        if self._session is None:
            timeout = aiohttp.ClientTimeout(total=10)
            self._session = aiohttp.ClientSession(timeout=timeout)
        return self._session

    async def plan_grasp(self, command: str) -> BrainPlan:
        if not command:
            raise ValueError('Command must be a non-empty string.')

        if self._endpoint:
            payload = {'command': command}
            session = await self._ensure_session()
            async with session.post(self._endpoint, json=payload) as resp:
                resp.raise_for_status()
                data: Dict[str, Any] = await resp.json()
                return BrainPlan(
                    command=command,
                    confidence=float(data.get('confidence', 0.75)),
                    gripper_width=float(data.get('gripper_width', 0.04)),
                    grasp_effort=float(data.get('grasp_effort', 40.0)),
                    reasoning=data.get('reasoning', 'Remote brain responded.')
                )

        # Fallback heuristic plan when no remote endpoint is configured.
        await asyncio.sleep(0)
        return BrainPlan(
            command=command,
            confidence=0.8,
            gripper_width=0.035,
            grasp_effort=45.0,
            reasoning='默认策略：抓取可乐罐需要收拢手指至 3.5cm, 施加 45N 力。'
        )
