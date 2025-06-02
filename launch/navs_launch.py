# Copyright 2025 Nicolás García Moncho

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_warehouse')

    nav2sim_cmd1 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim1',
        namespace='small_robot_1',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
    )

    nav2sim_cmd2 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim2',
        namespace='small_robot_2',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
    )

    nav2sim_cmd3 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim3',
        namespace='medium_robot_1',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
    )

    nav2sim_cmd4 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim4',
        namespace='medium_robot_2',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
    )

    nav2sim_cmd5 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim5',
        namespace='big_robot_1',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
    )

    nav2sim_cmd6 = Node(
        package='plansys2_warehouse',
        executable='nav2_sim_node',
        name='nav2sim6',
        namespace='big_robot_2',
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml']
    )

    ld.add_action(nav2sim_cmd1)
    ld.add_action(nav2sim_cmd2)
    ld.add_action(nav2sim_cmd3)
    ld.add_action(nav2sim_cmd4)
    ld.add_action(nav2sim_cmd5)
    ld.add_action(nav2sim_cmd6)

    return ld
