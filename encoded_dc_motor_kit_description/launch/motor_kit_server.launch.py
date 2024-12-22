import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration

package_name = "encoded_dc_motor_kit_description"
xacro_file = "encoded_dc_motor_kit.xacro"

def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory(package_name), "urdf", xacro_file)

    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = urdf_file,
        description = "Absolute path to robot urdf"
    )

    robot_description = ParameterValue(Command(
            ['xacro ', LaunchConfiguration("model")]
        )
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        arguments=[urdf_file],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    config_file_path = os.path.join( get_package_share_directory('encoded_dc_motor_kit_control'), 'config','controller.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, config_file_path],
        #parameters=[config_file_path],
        output="screen"
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen"
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
        output="screen"
    )

    rviz_config = os.path.join(get_package_share_directory("encoded_dc_motor_kit_description"), "rviz", "rviz_sim.rviz")

    rviz2 = Node(
        package="rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments=["-d", rviz_config]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    models_visualisation_node = Node(
        package = "encoded_dc_motor_kit_models",
        executable = "model_visualiser"
    )


    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        effort_controller_spawner,
        #velocity_controller_spawner,
        #rviz2,
        #joint_state_publisher_gui,
        #models_visualisation_node,
    ])