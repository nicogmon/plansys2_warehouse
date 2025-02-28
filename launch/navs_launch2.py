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
    
    nav2sim_cmd1 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim',
        namespace='small_robot',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
        )
    nav2sim_cmd2 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim',
        namespace='medium_robot',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
        )
    nav2sim_cmd3 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim',
        namespace='big_robot',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
        )
    # ld.add_action(nav2sim_cmd1)
    ld.add_action(nav2sim_cmd2)
    #ld.add_action(nav2sim_cmd3)

    ld.add_action(declare_namespace_cmd)
    
    return ld