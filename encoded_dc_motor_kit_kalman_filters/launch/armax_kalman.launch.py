#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_params = PathJoinSubstitution(
        [
            FindPackageShare("encoded_dc_motor_kit_kalman_filters"),
            "config",
            "armax_kalman.yaml",
        ]
    )

    cart_pole_lypunov_lqr_node = Node(
        package="encoded_dc_motor_kit_kalman_filters",
        executable="kalman_filter_with_arxmax_model",
        parameters=[controller_params],
    )

    actions = [
        cart_pole_lypunov_lqr_node,
    ]

    return LaunchDescription(actions)