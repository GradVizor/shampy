# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='shampy',
                          description='Ignition model name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='World name'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')


    pkg_shampy_simulator = get_package_share_directory(
        'shampy_simulator')

    create3_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_shampy_simulator, 'launch', 'create3_ros_ign_bridge.launch.py'])

    create3_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ros_ign_bridge_launch]),
        launch_arguments=[
            ('robot_name', robot_name),
            ('namespace', namespace),
            ('world', world)
        ]
    )

    # lidar bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/rplidar_link/sensor/rplidar/scan' +
             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/rplidar_link/sensor/rplidar/scan'],
             'scan')
        ])
    
    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            [namespace,
             '/cmd_vel' + '@geometry_msgs/msg/TwistStamped' + '[gz.msgs.Twist'],
            ['/model/', robot_name, '/cmd_vel' +
             '@geometry_msgs/msg/TwistStamped' +
             ']gz.msgs.Twist']
        ],
        remappings=[
            ([namespace, '/cmd_vel'], 'cmd_vel'),
            (['/model/', robot_name, '/cmd_vel'],
             'diffdrive_controller/cmd_vel')
        ])
    
    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ['/model/', robot_name, '/tf' +
                                    '@tf2_msgs/msg/TFMessage' +
                                    '[gz.msgs.Pose_V']
                               ],
                               remappings=[
                                   (['/model/', robot_name, '/tf'], 'tf')
                               ])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(lidar_bridge)
    # ld.add_action(cmd_vel_bridge)
    ld.add_action(odom_base_tf_bridge)
    return ld
