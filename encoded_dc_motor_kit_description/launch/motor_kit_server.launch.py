import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    use_gui_arg = DeclareLaunchArgument(
        name='use_gui',
        default_value='true',
        description='Flag to enable velocity command GUI'
    )

    use_plotjuggler_arg = DeclareLaunchArgument(
        name='use_plotjuggler',
        default_value='true',
        description='Flag to enable PlotJuggler GUI'
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

    config_file_path = os.path.join( get_package_share_directory('encoded_dc_motor_kit_control'), 'config','controller.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, config_file_path],
        output="screen"
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
        output="screen"
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )
    

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        effort_controller_spawner,
        OpaqueFunction(function=launch_gui_nodes),
        
    ])


def launch_gui_nodes(context):
    nodes = []
    if context.launch_configurations.get("use_gui", "false").lower() == "true":
        nodes.append(
            Node(
                package="encoded_dc_motor_kit_gui",
                executable="velocity_publisher_gui.py",
            )
        )
    if context.launch_configurations.get("use_plotjuggler", "false").lower() == "true":
        nodes.append(Node(package="plotjuggler", executable="plotjuggler"))
    return nodes