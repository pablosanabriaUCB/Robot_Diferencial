"""
micro_ros_agent.launch.py

ROS 2 Launch file that starts the micro-ROS agent via Docker.
The agent listens on UDP port 8888 for micro-ROS clients (ESP32).

Usage:
    ros2 launch micro_ros_agent.launch.py

Note: Requires Docker to be installed and the micro-ROS agent image
      to be available (microros/micro-ros-agent:jazzy).
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """Generate launch description for the micro-ROS agent."""

    micro_ros_agent = ExecuteProcess(
        cmd=[
            'sudo', 'docker', 'run',
            '-it', '--rm', '--net=host',
            'microros/micro-ros-agent:jazzy',
            'udp4', '--port', '8888', '-v6'
        ],
        name='micro_ros_agent',
        output='screen'
    )

    return LaunchDescription([
        micro_ros_agent
    ])
