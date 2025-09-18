from embodiedai_mvp.brain.brain_client import BrainClient
import pytest


@pytest.mark.asyncio
async def test_brain_client_fallback_plan():
    async with BrainClient(endpoint=None) as client:
        plan = await client.plan_grasp('拿起可乐')
    assert plan.gripper_width > 0
    assert plan.grasp_effort > 0
    assert '可乐' in plan.reasoning
