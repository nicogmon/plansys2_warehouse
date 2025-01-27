# Copyright 2019 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_warehouse_problem')
    print("example_dir: %s", example_dir)
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

    # Specify the actions

    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])
    load_box_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='load_box',
        namespace=namespace,
        output='screen',
        parameters=[   
          example_dir + '/config/params.yaml',{
            'action_name': 'load_box',
            'publisher_port': 1670,
            'server_port': 1671,
            'bt_xml_file': example_dir + '/behavior_trees_xml/load_box.xml'
          }    
        ])
             
    unload_box_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='unload_box',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'unload_box',
            'publisher_port': 1672,
            'server_port': 1673,
            'bt_xml_file': example_dir + '/behavior_trees_xml/unload_box.xml'
          }
        ])
    
    
    # controller_cmd = Node(
    #     package='plansys2_house_problem',
    #     executable='house_controller_node',
    #     name='house_controller',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[example_dir + '/config/params.yaml']
    #     )
    

    
    
    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)
    
    

    ld.add_action(move_cmd)
    ld.add_action(load_box_cmd)
    ld.add_action(unload_box_cmd)

    # ld.add_action(controller_cmd)

    
    
    
    return ld
