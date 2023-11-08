# Copyright 2023 ICube Laboratory, University of Strasbourg
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='hk1d_bringup_master_follower',
            description='Config package with the various runtime config files (ros2 controllers, gazebo, etc.).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint/link names, useful for multi-robot setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='force',
            description='Robot command interface [position|velocity|effort].',
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    command_interface = LaunchConfiguration('command_interface')
    description_package = "hk1d_description"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('hk1d_bringup_master_follower'), 'config', 'hk1d.config.xacro']
            ),
            ' ',
            'prefix:=', prefix,
            ' ',
            'namespace:=', namespace,
            ' ',
            'use_sim:=', use_sim,
            ' ',
            'use_fake_hardware:=', use_fake_hardware,
            ' ',
            'command_interface:=', command_interface,
            ' ',
            'description_package:=', description_package,
            ' ',
            'runtime_config_package:=', runtime_config_package,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    def get_robot_description_semantic_content():
        return Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "srdf", "hk1d.srdf.xacro"]
            ),
            " ",
            "name:=", "iiwa",
            " ",
            "prefix:=", prefix,
            " ",
            'description_package:=', description_package,
            ' ',
            'runtime_config_package:=', runtime_config_package,
        ])

    # robot_description_semantic = {
    #     'robot_description_semantic': get_robot_description_semantic_content()
    # }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hk1d_description"), "rviz", "hk1d.rviz"]
    )

    # logger = launch.substitutions.LaunchConfiguration("log_level");

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            'config',
            'hk1d_controllers.yaml',
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # parameters=[]
        parameters=[robot_description, robot_controllers],
        output="both",
        # arguments=['--ros-args', '--log-level', "debug"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # arguments=['--ros-args', '--log-level', "debug"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
