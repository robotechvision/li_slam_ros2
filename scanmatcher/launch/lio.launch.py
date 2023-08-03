import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mapping_param_dir = launch.substitutions.LaunchConfiguration(
        'mapping_param_dir',
        default=os.path.join(
            get_package_share_directory('scanmatcher'),
            'param',
            'lio.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[mapping_param_dir],
        # prefix="gnome-terminal -- gdb -ex run --args",
        remappings=[('/input_cloud','/noground_points')],
        output='screen'
        )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        # prefix="gnome-terminal -- gdb -ex run --args",
        parameters=[mapping_param_dir],
        output='screen'
        )

    # tf = launch_ros.actions.Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0','0','0','0','0','0','1','base_link','velodyne']
    #     )
    #
    # imu_pre = launch_ros.actions.Node(
    #     package='scanmatcher',
    #     executable='imu_preintegration',
    #     remappings=[('/odometry','/ekf_odom/odom')],
    #     parameters=[mapping_param_dir],
    #     output='screen'
    #     )
    #
    # img_pro = launch_ros.actions.Node(
    #     package='scanmatcher',
    #     executable='image_projection',
    #     parameters=[mapping_param_dir],
    #     output='screen'
    #     )

    ground_filter =  launch_ros.actions.Node(
        package='rtv_ground_height',
        executable='grid_interp_ground_height',
        parameters=[mapping_param_dir],
        remappings=[
            ('/points_in', '/velodyne_points'),
            ('/points_out', '/noground_points'),
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mapping_param_dir',
            default_value=mapping_param_dir,
            description='Full path to mapping parameter file to load'),
        mapping,
        # tf,
        # imu_pre,
        # img_pro,
        graphbasedslam,
        ground_filter,
        launch_ros.actions.Node(
            package='rtv_lifecycle',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[mapping_param_dir,
                        {
                            'autostart': False,
                            'bond_timeout': 100000.0,
                            'node_names': ['graph_based_slam', 'scan_matcher', 'grid_interp_ground_height']
                        }]),
            ])