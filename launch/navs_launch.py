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
