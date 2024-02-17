import os

from ament_index_python.packages import get_package_share_directory

from launch import conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ig_lio_c_dir = get_package_share_directory('ig_lio_c')
    config = os.path.join(ig_lio_c_dir, 'config', 'parameters.yaml')

    ig_lio_c_node = Node(
        package='ig_lio_c',
        executable='map_builder_node',
        name='map_builder_node',
        output='screen',
        parameters=[config]
    )

    imu_complementary_filter_node = Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                ],
            )

    ld = LaunchDescription()

    ld.add_action(ig_lio_c_node)
    ld.add_action(imu_complementary_filter_node)

    return ld