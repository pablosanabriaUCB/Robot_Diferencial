#!/bin/bash
# ============================================================
# run_agent.sh
# Launches the micro-ROS agent via Docker container.
# Transport: UDP on port 8888, verbose level 6.
#
# Usage:
#   chmod +x run_agent.sh
#   ./run_agent.sh
# ============================================================

sudo docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
