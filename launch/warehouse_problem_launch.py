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

    
    robot_names = ["small_robot", "med_robot", "big_robot"]
    move_nodes = {} 
    load_nodes = {}
    unload_nodes = {}

    for i, robot_name in enumerate(robot_names, start=1):
        move_nodes[f"move_node_{robot_name}"] = Node(
            package='plansys2_warehouse',
            executable='move_node',
            name=f"move{i}",
            namespace='',#preguntar a fran por el namesapce igual esto nos ayuda a la hora de que lo acccepte el action executor correcto
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
            name=f"load_box{i}",
            namespace='',
            output='screen',
            parameters=[])
        
        unload_nodes[f"unload_node_{robot_name}"] = Node(
            package='plansys2_warehouse',
            executable='unload_node',
            name=f"unload_box{i}",
            namespace='',
            output='screen',
            parameters=[])
        

        ld.add_action(move_nodes[f"move_node_{robot_name}"])
        ld.add_action(load_nodes[f"load_node_{robot_name}"])
        ld.add_action(unload_nodes[f"unload_node_{robot_name}"])
    
    
    
    # controller_cmd = Node(
    #     package='plansys2_house_problem',
    #     executable='house_controller_node',
    #     name='house_controller',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[example_dir + '/config/params.yaml']
    #     )
    

    
    
    

    # Create the launch description and populate
    

    # Declare the launch options
    
    

    # ld.add_action(controller_cmd)

    
    
    
    return ld
