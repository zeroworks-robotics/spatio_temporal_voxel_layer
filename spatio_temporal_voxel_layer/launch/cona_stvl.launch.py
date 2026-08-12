# Standalone STVL costmap for the CoNA rig, includable from another launch file.
#
# nav2_costmap_2d is a lifecycle node: launching it on its own leaves it UNCONFIGURED and
# silent, which reads as "started fine, publishes nothing". nav2_lifecycle_manager with
# autostart drives it through configure/activate, so this needs no shell wrapper and can
# be pulled into a bringup with IncludeLaunchDescription.
#
# Two things are not free parameters:
#   * The node name. nav2_costmap_2d's standalone main ignores __node/__ns and always
#     comes up as /costmap/costmap, so the params-file key and the manager's node_names
#     entry both have to say that.
#   * bond. The costmap node does not create a bond with the manager, so bond_timeout
#     must be 0.0 or the manager decides it died and tears it back down.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

COSTMAP_NODE = '/costmap/costmap'


def generate_launch_description():
    pkg = get_package_share_directory('spatio_temporal_voxel_layer')
    default_params = os.path.join(pkg, 'example', 'cona_stvl_costmap.yaml')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    set_dds = LaunchConfiguration('set_cyclonedds_uri')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Costmap + STVL parameters.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Drive the costmap through configure/activate on start.'),
        DeclareLaunchArgument(
            # Only useful when launched by hand from a plain shell. Included from the
            # robot bringup the environment is already right, and overwriting it there
            # would be wrong, so this defaults to off in that case by being explicit.
            'set_cyclonedds_uri', default_value='false',
            description='Export the robot CYCLONEDDS_URI. Off when the parent already set it.'),

        SetEnvironmentVariable(
            'CYCLONEDDS_URI', 'file:///etc/coga-robotics/conf/cyclonedds.xml',
            condition=IfCondition(set_dds)),

        Node(
            package='nav2_costmap_2d', executable='nav2_costmap_2d',
            name='costmap', namespace='costmap',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_stvl', output='screen',
            parameters=[{
                'autostart': autostart,
                'node_names': [COSTMAP_NODE],
                # The costmap node creates no bond, so a non-zero timeout makes the
                # manager conclude it crashed and shut it down again a few seconds in.
                'bond_timeout': 0.0,
            }],
        ),
    ])
