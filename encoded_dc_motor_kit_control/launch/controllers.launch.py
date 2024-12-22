from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    position_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "position_controller",
                "--controller-manager", "/controller_manager",
            ]

    )

    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broad",
            ]
        )
    
    velocity_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "velocity_controller",
                "--controller-manager", "/controller_manager",
            ]
    )

    return LaunchDescription(
        [
            joint_state_broadcaster,
            position_controller,
            # velocity_controller, 
        ]
    )
