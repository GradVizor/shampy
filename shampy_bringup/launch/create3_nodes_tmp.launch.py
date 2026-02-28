#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 nodes

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('gazebo', default_value='classic',
                          choices=['classic', 'ignition'],
                          description='Which gazebo simulator to use'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():

    # Directories
    pkg_create3_control = get_package_share_directory('shampy_control')
    pkg_shampy_nodes = 'shampy_nodes'
    
    # Paths
    control_launch_file = PathJoinSubstitution(
        [pkg_create3_control, 'src', 'control.py'])

    # Includes
    diffdrive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_launch_file]),
        launch_arguments=[('namespace', LaunchConfiguration('namespace'))]
    )

    # Motion Control
    motion_control_node = Node(
        package=pkg_shampy_nodes,
        name='motion_control',
        executable='motion_control',
        parameters=[{
            'use_sim_time': True,
            'safety_override': 'backup_only'
        }],
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )


    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Include robot description
    ld.add_action(diffdrive_controller)
    ld.add_action(motion_control_node)
    return ld
