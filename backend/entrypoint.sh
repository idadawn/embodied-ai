#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

# Use provided config if mounted, else fall back to default inside image
export PYTHONUNBUFFERED=1

echo "[entrypoint] Launching EmbodiedAI MVP stack..."
exec ros2 launch embodiedai_mvp sim.launch.py

