from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # -------------------------
    # Launch Configurations
    # -------------------------
    component_container = LaunchConfiguration('component_container')
    points_topic = LaunchConfiguration('points_topic')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id')

    use_imu = LaunchConfiguration('use_imu'),
    imu_linear_acc_unit_g = LaunchConfiguration('imu_linear_acc_unit_g'),
    invert_imu_acc = LaunchConfiguration('invert_imu_acc')
    invert_imu_gyro = LaunchConfiguration('invert_imu_gyro')
    use_global_localization = LaunchConfiguration('use_global_localization')
    imu_topic = LaunchConfiguration('imu_topic')
    enable_robot_odometry_prediction = LaunchConfiguration('enable_robot_odometry_prediction')
    robot_odom_frame_id = LaunchConfiguration('robot_odom_frame_id')
    plot_estimation_errors = LaunchConfiguration('plot_estimation_errors')
    globalmap_pcd = LaunchConfiguration('globalmap_pcd')

    # -------------------------
    # Global Localization Include
    # -------------------------
    global_localization_node = Node(
        package='hdl_global_localization',
        executable='hdl_global_localization_node',
        name='hdl_global_localization_node',
        output='screen',
        parameters=[{
            # Add your parameters here if needed
        }]
    )

    # -------------------------
    # Component Container (Component Manager replacement)
    # -------------------------
    container = ComposableNodeContainer(
        name=component_container,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[

            # -------------------------
            # Globalmap Server Component
            # -------------------------
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::GlobalmapServerComponent',
                name='globalmap_server',
                parameters=[{
                    'globalmap_pcd': globalmap_pcd,
                    'convert_utm_to_local': True,
                    'downsample_resolution': 0.2,
                }]
            ),

            # -------------------------
            # HDL Localization Component
            # -------------------------
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::HdlLocalizationComponent',
                name='hdl_localization',
                remappings=[
                    ('/velodyne_points', points_topic),
                    ('/gpsimu_driver/imu_data', imu_topic),
                ],
                parameters=[{
                    # frames
                    'odom_child_frame_id': odom_child_frame_id,

                    # imu
                    'use_imu': use_imu,
                    'imu_linear_acc_unit_g': imu_linear_acc_unit_g,
                    'invert_acc': invert_imu_acc,
                    'invert_gyro': invert_imu_gyro,
                    'cool_time_duration': 2.0,

                    # odometry prediction
                    'enable_robot_odometry_prediction': enable_robot_odometry_prediction,
                    'robot_odom_frame_id': robot_odom_frame_id,

                    # ndt
                    'reg_method': 'NDT_OMP',
                    'ndt_neighbor_search_method': 'DIRECT7',
                    'ndt_neighbor_search_radius': 2.0,
                    'ndt_resolution': 1.0,
                    'downsample_resolution': 0.2,

                    # init pose
                    'specify_init_pose': True,
                    # 'init_pos_x': -6.728906154632568,
                    # 'init_pos_y': 18.065521240234375,
                    'init_pos_x': 0.0,
                    'init_pos_y': 0.0,
                    'init_pos_z': 1.1,
                    'init_ori_w': 1.0,
                    'init_ori_x': 0.0,
                    'init_ori_y': 0.0,
                    'init_ori_z': 0.0,

                    'use_global_localization': use_global_localization,
                }]
            ),
        ]
    )

    # -------------------------
    # Optional Plot Node
    # -------------------------
    plot_node = Node(
        package='hdl_localization',
        executable='plot_status.py',
        name='plot_estimation_errors',
        condition=IfCondition(plot_estimation_errors),
        output='screen'
    )

    # -------------------------
    # Declare Arguments
    # -------------------------
    return LaunchDescription([

        DeclareLaunchArgument('component_container', default_value='hdl_localization_container'),
        DeclareLaunchArgument('points_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('odom_child_frame_id', default_value='base_link'),

        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('imu_linear_acc_unit_g', default_value='true'),
        DeclareLaunchArgument('invert_imu_acc', default_value='false'),
        DeclareLaunchArgument('invert_imu_gyro', default_value='false'),
        DeclareLaunchArgument('use_global_localization', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('enable_robot_odometry_prediction', default_value='false'),
        DeclareLaunchArgument('robot_odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('plot_estimation_errors', default_value='false'),

        DeclareLaunchArgument('globalmap_pcd', default_value='/home/unitree/markov/nav_ws/install/humanoid_nav2_bringup/share/humanoid_nav2_bringup/maps/2nd_floor/pc/localization_cloud.pcd'),

        global_localization_node,
        container,
        # plot_node
    ])