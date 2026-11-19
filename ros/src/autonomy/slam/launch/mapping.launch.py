import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_dir = os.path.dirname(os.path.dirname(__file__))
    robot_xacro = os.path.normpath(os.path.join(slam_dir, '..', '..', 'bringup', 'description', 'pioneer3at_lidar.xacro'))
    camera_launch = os.path.normpath(os.path.join(slam_dir, '..', '..', 'hardware', 'camera', 'launch', 'camera.launch.py'))
    subscribe_scan_cloud = LaunchConfiguration('subscribe_scan_cloud')
    use_rgbd = LaunchConfiguration('use_rgbd')
    use_lidar_scan = ["'", subscribe_scan_cloud, "'.lower() in ('true', '1')"]
    use_femto_imu = ["'", use_rgbd, "'.lower() in ('true', '1') and not (", *use_lidar_scan, ")"]
    femto_imu_topic = '/camera/imu/data'
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', robot_xacro,
                                                ' lidar_x:=', LaunchConfiguration('lidar_x'), ' lidar_y:=', LaunchConfiguration('lidar_y'), ' lidar_z:=', LaunchConfiguration('lidar_z'),
                                                ' lidar_roll:=', LaunchConfiguration('lidar_roll'), ' lidar_pitch:=', LaunchConfiguration('lidar_pitch'), ' lidar_yaw:=', LaunchConfiguration('lidar_yaw')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('pioneer_port', default_value='/dev/pioneer'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/unilidar'),
        DeclareLaunchArgument('lidar_x', default_value='0'),
        DeclareLaunchArgument('lidar_y', default_value='0'),
        DeclareLaunchArgument('lidar_z', default_value='0.225'),
        DeclareLaunchArgument('lidar_roll', default_value='0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0'),
        DeclareLaunchArgument('database_path', default_value=os.path.join(slam_dir, "maps", "temp_map.db")), #later should be changed to some smart naming depending on the place we are
        DeclareLaunchArgument('odom_frame_id', default_value=''),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('subscribe_scan_cloud', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value=subscribe_scan_cloud),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('use_rgbd', default_value='false'),
        DeclareLaunchArgument('filter', default_value='deskew', choices=['object', 'deskew', 'none']),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('p2os_bringup'), 'launch', 'p2os_driver_launch.py'])),
            launch_arguments={'port': LaunchConfiguration('pioneer_port')}.items(),
            condition=IfCondition(LaunchConfiguration('start_base')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'publish_frequency': 30.0, 'tf_prefix': ''}],
            output='screen',
        ),
        Node(
            package='p2os_urdf',
            executable='p2os_publisher_3at',
            name='wheel_publisher',
            output='screen',
        ),
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            parameters=[{'port': LaunchConfiguration('lidar_port'), 'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            output='screen',
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch), condition=IfCondition(LaunchConfiguration('start_camera'))),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='femto_imu_filter',
            parameters=[{'use_mag': False, 'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/gyro_accel/sample'), ('imu/data', femto_imu_topic)],
            condition=IfCondition(PythonExpression(use_femto_imu)),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_imu_mount',
            arguments=[
                '--x', '-0.007698', '--y', '-0.014655', '--z', '0.00667',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'unilidar_lidar', '--child-frame-id', 'unilidar_imu',
            ],
            output='screen',
        ),
        Node( # deskewing the point cloud to account for the motion of the robot during the scan
            package='rtabmap_util',
            executable='lidar_deskewing',
            name='lidar_deskewing',
            parameters=[{'fixed_frame_id': 'odom', 'wait_for_transform': 0.2}],  # the base's odom TF is only 10 Hz; the 0.01 s default made every scan fail
            remappings=[('input_cloud', '/unilidar/cloud')],
            condition=IfCondition(PythonExpression(use_lidar_scan + [" and '", LaunchConfiguration('filter'), "' != 'none'"])),
            output='screen',
        ),
        ExecuteProcess( # filtering out dynamic objects from the point cloud such as moving people
            cmd=[
                'python3', os.path.join(slam_dir, 'dynamic_object_filter.py'),
                '--pointcloud-topic', '/unilidar/cloud/deskewed',
                '--output-topic', '/unilidar/cloud_static',
                '--algorithm', 'range',
                '--fixed-frame', 'odom',
                '--range-window', '3',
            ],
            condition=IfCondition(PythonExpression(use_lidar_scan + [" and '", LaunchConfiguration('filter'), "' == 'object'"])),
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_dir, 'launch', 'slam.launch.py')),
            launch_arguments={
                'database_path': LaunchConfiguration('database_path'),
                'config_file': os.path.join(slam_dir, 'config', 'rtabmap_hardware.yaml'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'scan_cloud_topic': PythonExpression(["'/unilidar/cloud_static' if '", LaunchConfiguration('filter'), "' == 'object' else '/unilidar/cloud/deskewed' if '", LaunchConfiguration('filter'), "' == 'deskew' else '/unilidar/cloud'"]),
                'imu_topic': PythonExpression(["'/unilidar/imu' if ", *use_lidar_scan, f" else '{femto_imu_topic}' if ", *use_femto_imu, " else ''"]),
                'use_rgbd': use_rgbd,
                'subscribe_scan_cloud': subscribe_scan_cloud,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(slam_dir, 'config', 'mapping.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
