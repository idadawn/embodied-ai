FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-setuptools python3-wheel git \
    libgl1 libglib2.0-0 libglfw3 \
    ros-humble-control-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-example-interfaces \
    ros-humble-rmw-cyclonedds-cpp \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy backend package
COPY . /app

# Install Python deps and the package (rclpy provided by ROS apt, not pip)
RUN pip3 install --no-cache-dir -U pip \
  && pip3 install --no-cache-dir -e .

# Entrypoint sets up ROS and launches all nodes + MCP
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8080 9876

ENTRYPOINT ["/entrypoint.sh"]

