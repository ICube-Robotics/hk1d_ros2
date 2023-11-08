# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from launch import LaunchContext

def generate_launch_description():

    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'isReal_Slave',
            default_value='false',
            description='if true, a real slave robot will be expected, otherwise, a simulated one will be launched',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'isReal_Master',
            default_value='false',
            description='if true, a real Master robot will be expected, otherwise, a simulated one will be launched',
        )
    )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'control_freq',
    #         default_value='800',
    #         description='control frequency, will be passed to hardware controller in (hk1d.r2c_hardware.xacro)',
    #     )
    # )

    # Initialize Arguments
    isReal_Slave = LaunchConfiguration('isReal_Slave')
    isReal_Master = LaunchConfiguration('isReal_Master')

    # control_freq = LaunchConfiguration('control_freq')

    lc = LaunchContext()
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hk1d_bringup"),
            "config",
            "hk1d_controllers.yaml",
        ]
    )

    with open(robot_controllers.perform(lc)) as info:
      info_dict = yaml.load(info,yaml.FullLoader)
      print("update rate is:", info_dict['controller_manager']['ros__parameters']['update_rate'])
    set_membership_params = info_dict['ar_mpc_controller']['ros__parameters']
    control_freq = info_dict['controller_manager']['ros__parameters']['update_rate']
    Ts = 1/control_freq

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hk1d_description"),
                    "urdf",
                    "hk1d.config.xacro",
                ]
            ),
            ' ',
            'isReal_Slave:=',
            isReal_Slave,
            ' ',
            'isReal_Master:=',
            isReal_Master,
            ' ',
            'control_freq:=',
            str(control_freq),
            ' ',
            'Ts:=',
            str(Ts),
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hk1d_description"), "rviz", "hk1d.rviz"]
    )

    # logger = launch.substitutions.LaunchConfiguration("log_level");


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # parameters=[]
        parameters=[robot_description, robot_controllers,{'isReal_Slave': isReal_Slave},{'isReal_Master': isReal_Master},{'control_freq':str(control_freq)},{'Ts':Ts}],
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

    armpc_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ar_mpc_controller", "-c", "/controller_manager"],
    )  

    simple_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["teleop_sim2sim_simple_controller", "-c", "/controller_manager"],
    )  

    plotJuggler_layout_file = [
            PathJoinSubstitution(
                [
                    FindPackageShare("hk1d_bringup"),
                    "config",
                    "PlotJuggler_layout.xml",
                ]
            ),
        ]
    plotJuggler_Node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["-l",plotJuggler_layout_file],
    )  

    set_membership_params = PathJoinSubstitution(
        [
            FindPackageShare("set_membership"),
            "config",
            "parameters.yaml",
        ]
    )

    node_set_membership = Node(
        package="set_membership",
        executable="set_membership_node",
        # namespace="",
        # prefix=["gdbserver localhost:3000"], #for debugging
        output="screen",
        parameters=[set_membership_params,{'Ts':Ts}]
        # parameters=[set_membership_params,{'Ts':Ts}],
    )
    print("Launch file Ts = ", Ts)

    teleop_real2sim_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["teleop_real2sim_controller", "-c", "/controller_manager"],
    )  

    teleop_real2real_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["teleop_real2real_controller", "-c", "/controller_manager"],
    ) 

    teleop_sim2sim_simple_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["teleop_sim2sim_simple_controller", "-c", "/controller_manager"],
    ) 

    if (not isReal_Master) and (not isReal_Slave):
       controller_Node = teleop_sim2sim_simple_controller_spawner
    elif(isReal_Master) and (not isReal_Slave):
       controller_Node = teleop_real2sim_controller_spawner
    else:
    #    controller_Node = teleop_real2real_controller_spawner
       controller_Node = armpc_controller_spawner
        

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # node_set_membership,
        ## simple_controller_spawner,
        ## plotJuggler_Node,
        ## rviz_node,
        controller_Node,
       
    ]

    return LaunchDescription(declared_arguments + nodes)
