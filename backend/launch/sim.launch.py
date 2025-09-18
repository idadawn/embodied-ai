"""Launch description for the EmbodiedAI MVP simulation stack."""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    base_path = Path(__file__).resolve().parents[1]
    default_config = str(base_path / 'config' / 'app.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the YAML configuration file.'
    )

    config = LaunchConfiguration('config')

    sim_node = Node(
        package='embodiedai-mvp',
        executable='embodiedai-sim',
        name='simulation_node',
        output='screen',
        parameters=[{'config_path': config}],
    )

    vision_node = Node(
        package='embodiedai-mvp',
        executable='embodiedai-vision',
        name='vision_node',
        output='screen',
        parameters=[{'config_path': config}],
    )

    grasp_node = Node(
        package='embodiedai-mvp',
        executable='embodiedai-grasp',
        name='grasp_action_server',
        output='screen',
        parameters=[{'config_path': config}],
    )

    mcp_node = Node(
        package='embodiedai-mvp',
        executable='embodiedai-mcp',
        name='mcp_server',
        output='screen',
        arguments=['--config', config],
    )

    return LaunchDescription([config_arg, sim_node, vision_node, grasp_node, mcp_node])
