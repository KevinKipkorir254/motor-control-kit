#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    try:
      config_path = os.path.join(
            get_package_share_directory("encoded_dc_motor_kit_model_predictive_control"),
            "config",
            "model.yaml"
        )
      if os.path.exists(config_path):
            print(f"[INFO] Configuration file found: {config_path}")
      else:
            print(f"[ERROR] Configuration file not found at: {config_path}")
    except Exception as e:
            print(f"[ERROR] Failed to check configuration path: {e}")
        
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("encoded_dc_motor_kit_model_predictive_control"),
            "config",
            "model.yaml",
        ]
    )
    

    cart_pole_lypunov_lqr_node = Node(
        package="encoded_dc_motor_kit_model_predictive_control",
        executable="mpc_controller",
        parameters=[controller_params],
    )

    actions = [
        cart_pole_lypunov_lqr_node,
    ]

    return LaunchDescription(actions)