import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_bot'
    
    # 1. URDF(Xacro) 변환 - 로봇 설계도를 읽어옵니다 (절대 생략 불가!)
    file_subpath = 'urdf/robot_urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. 월드 파일 경로 설정 (새로 만든 mapping_office.sdf)
    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'mapping_office.sdf')

    # 3. Gazebo 실행 (새 월드 적용)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 4. 로봇 소환
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.1'],
        output='screen'
    )

    # 5. Robot State Publisher (로봇 형태 유지)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 6. ROS-Gazebo Bridge (월드 이름 'empty' 유지)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/empty/model/my_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/my_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/world/empty/model/my_bot/joint_state', '/joint_states'),
            ('/model/my_bot/tf', '/tf')
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan'
        }]
    )

    # 8. Static TF Publisher (라이다 프레임 연결)
    node_static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0', 
                   '--frame-id', 'laser_frame', 
                   '--child-frame-id', 'my_bot/base_link/lidar'],
        parameters=[{'use_sim_time': True}]
    )

    delayed_spawn_entity = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        bridge,
        slam_toolbox,
        node_static_tf_lidar,
        delayed_spawn_entity
    ])