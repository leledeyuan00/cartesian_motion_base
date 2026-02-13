from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs) -> list[Node]:

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    calibration_file = LaunchConfiguration("calibration_file")

    robot_camera_matrix = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            calibration_file,
        ]
    )
    motion_test_node = Node(
        package=runtime_config_package,
        executable="cartesian_motion_dual_arm_test",
        parameters=[
            robot_camera_matrix,
        ],
        output="screen",
    )

    nodes_to_start: list[Node] = [motion_test_node]

    return nodes_to_start


def generate_launch_description() -> LaunchDescription:

    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the \
            controllers' configuration have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="cartesian_motion_test",
            description='Package with the controller\'s configuration \
            in "config" folder. Usually the argument is not set, it \
            enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "calibration_file",
            default_value="robot_camera.yaml",
            description="YAML file with the calibration configuration.",
        )
    )

    return LaunchDescription(declared_arguments +
                             [OpaqueFunction(function=launch_setup)])
