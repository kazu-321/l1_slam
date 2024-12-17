import os
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # Run unitree lidar
  node1 = Node(
    package='unitree_lidar_ros2',
    executable='unitree_lidar_ros2_node',
    name='unitree_lidar_ros2_node',
    output='screen',
    parameters= [
              {'port': '/dev/ttyUSB0'},
              {'rotate_yaw_bias': 0.0},
              {'range_scale': 0.001},
              {'range_bias': 0.0},
              {'range_max': 50.0},
              {'range_min': 0.0},
              {'cloud_frame': "unilidar_lidar"},
              {'cloud_topic': "points2"},
              {'cloud_scan_num': 18},
              {'imu_frame': "unilidar_imu"},
              {'imu_topic': "imu"}]
  )

  return LaunchDescription([node1])
