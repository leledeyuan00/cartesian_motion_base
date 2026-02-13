# Copyright 2025 Smart Robotics Design Lab. All rights reserved.
#
# Licensed under the MIT License. See LICENSE file in the project root
# for license information.
# Author: Dayuan

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="cartesian_sim_description",
            description="Description package with robot URDF/xacro files. \
            Usually the argument is not set, it enables use of a custom \
            description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Start robot in Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="cartesian_compliance_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller_L",
            default_value="left_cartesian_compliance_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller_R",
            default_value="right_cartesian_compliance_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_type",
            default_value="single_arm",
            description="Selected the type of single or dual arms.",
        )
    )
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    robot_controller = LaunchConfiguration("robot_controller")
    robot_controller_L = LaunchConfiguration("robot_controller_L")
    robot_controller_R = LaunchConfiguration("robot_controller_R")
    config_type = LaunchConfiguration("config_type")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf",
                    description_file]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "config_type:=",
            config_type,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
        parameters=[robot_description],
        condition=IfCondition(
            PythonExpression(["'", config_type, "' == 'single_arm'"]))
    )

    robot_controller_spawner_L = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller_L, "-c", "/controller_manager"],
        parameters=[robot_description],
        condition=IfCondition(
            PythonExpression(["'", config_type, "' == 'dual_arm'"]))
    )

    robot_controller_spawner_R = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller_R, "-c", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(["'", config_type, "' == 'dual_arm'"]))
    )

    # Gazebo
    gazebo_world_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "gazebo", "ur_world.sdf"])

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [' -r -v 4 ', gazebo_world_file])],
        condition=IfCondition(sim_gazebo),
    )
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name',  'UR'],
        output='screen',
        condition=IfCondition(sim_gazebo),
    )

    nodes = [
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        robot_controller_spawner_L,
        robot_controller_spawner_R,
        # For Gazebo
        gazebo_node,
        gz_spawn_entity,
    ]

    return LaunchDescription(declared_arguments + nodes)
