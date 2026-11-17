import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_rtabmap(context):
    slam_dir = os.path.dirname(os.path.dirname(__file__))
    database_path = LaunchConfiguration('database_path').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    use_rgbd = LaunchConfiguration('use_rgbd').perform(context).lower() in ('true', '1')

    if mode == 'localisation' and not os.path.isfile(database_path):
        raise RuntimeError(f'No saved map at {database_path}')

    runtime_parameters = {
        'database_path': database_path,
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'odom_frame_id': ParameterValue(LaunchConfiguration('odom_frame_id'), value_type=str),
        'subscribe_rgbd': use_rgbd,
        'Mem/IncrementalMemory': 'true' if mode == 'mapping' else 'false',
    }

    if use_rgbd:
        runtime_parameters['qos_image'] = 2

    parameters = [os.path.join(slam_dir, 'config', 'rtabmap.yaml'), LaunchConfiguration('config_file'), runtime_parameters]

    if mode == 'localisation':
        parameters.append({
            'Mem/InitWMWithAllNodes': 'true',
            'initial_pose': ParameterValue(LaunchConfiguration('initial_pose'), value_type=str),
        })

    remappings = [
        ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
        ('odom', LaunchConfiguration('odom_topic')),
    ]
    imu_topic = LaunchConfiguration('imu_topic').perform(context)

    if imu_topic:
        remappings.append(('imu', imu_topic))

    nodes = []
    if use_rgbd:
        remappings.append(('rgbd_image', '/camera/rgbd_image'))
        nodes.append(Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool), 'qos': 2, 'approx_sync_max_interval': 0.01}],
            remappings=[('rgb/image', '/camera/color/image_raw'), ('depth/image', '/camera/depth_registered/image_rect'),
                        ('rgb/camera_info', '/camera/color/camera_info'), ('rgbd_image', '/camera/rgbd_image')],
            output='screen',
        ))

    nodes.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=parameters,
        remappings=remappings,
        output='screen',
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localisation']),
        DeclareLaunchArgument('initial_pose', default_value='0 0 0 0 0 0'),
        DeclareLaunchArgument('database_path'),
        DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/unilidar/cloud'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value=''),
        DeclareLaunchArgument('odom_frame_id', default_value=''),
        DeclareLaunchArgument('use_rgbd', default_value='false'),
        OpaqueFunction(function=launch_rtabmap),
    ])
