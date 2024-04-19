import os

from ament_index_python.packages import get_package_share_directory

from launch import conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    ig_lio_c_dir = get_package_share_directory('ig_lio_c')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = os.path.join(ig_lio_c_dir, 'config', 'parameters.yaml')

    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = RewrittenYaml(
        source_file=config,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    ig_lio_c_node = Node(
        package='ig_lio_c',
        executable='map_builder_node',
        name='map_builder_node',
        output='screen',
        parameters=[configured_params],
    )

    occupancy_grid_converter = Node(
        package='ig_lio_c',
        executable='occupancy_grid_converter',
        name='occupancy_grid_converter',
        output='screen',
        parameters=[configured_params],
    )

    # imu_complementary_filter_node = Node(
    #             package='imu_complementary_filter',
    #             executable='complementary_filter_node',
    #             name='complementary_filter_gain_node',
    #             output='screen',
    #             parameters=[
    #                 {'do_bias_estimation': True},
    #                 {'do_adaptive_gain': True},
    #                 {'use_mag': False},
    #                 {'gain_acc': 0.01},
    #                 {'gain_mag': 0.01},
    #             ],
    #         )

    ld = LaunchDescription([DeclareLaunchArgument('namespace', default_value='',
                            description='Top-level namespace'),
                            DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation (Gazebo) clock if true')])

    

    # ld.add_action(imu_complementary_filter_node)
    ld.add_action(ig_lio_c_node)
    ld.add_action(occupancy_grid_converter)

    return ld