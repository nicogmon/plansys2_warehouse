#  Copyright (C) 2025 Nicolás García Moncho

#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.

#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_warehouse')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': example_dir + '/pddl/warehouse_domain.pddl',
            'namespace': namespace
            }.items()
        )

    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)

    robot_names = ["small_robot_1", "medium_robot_1", "big_robot_1",
                   "small_robot_2", "medium_robot_2", "big_robot_2"]
    move_nodes = {}
    load_nodes = {}
    unload_nodes = {}

    for i, robot_name in enumerate(robot_names, start=1):
        move_nodes[f"move_node_{robot_name}"] = Node(
            package='plansys2_warehouse',
            executable='move_node',
            name=f"move_{robot_name}",
            namespace='',
            output='screen',
            parameters=[
                example_dir + '/config/params.yaml',
                {
                    'action_name': 'move',
                    'publisher_port': 1668,
                    'server_port': 1669
                }
            ])

        load_nodes[f"load_node_{robot_name}"] = Node(
            package='plansys2_warehouse',
            executable='load_node',
            name=f"load_box_{robot_name}",
            namespace='',
            output='screen',
            parameters=[
                example_dir + '/config/params.yaml',
                {
                    'action_name': f"load_box_{robot_name}",
                }])

        unload_nodes[f"unload_node_{robot_name}"] = Node(
            package='plansys2_warehouse',
            executable='unload_node',
            name=f"unload_box_{robot_name}",
            namespace='',
            output='screen',
            parameters=[
                example_dir + '/config/params.yaml',
                {
                    'action_name': f"unload_box_{robot_name}",
                }]
        )

        ld.add_action(move_nodes[f"move_node_{robot_name}"])
        ld.add_action(load_nodes[f"load_node_{robot_name}"])
        ld.add_action(unload_nodes[f"unload_node_{robot_name}"])

    goal_reader_cmd = Node(
        package='plansys2_warehouse',
        executable='goal_reader_node',
        name='goal_reader',
        namespace='',
        output='screen',
        parameters=[example_dir + '/config/params.yaml']
        )
    ld.add_action(goal_reader_cmd)

    return ld
