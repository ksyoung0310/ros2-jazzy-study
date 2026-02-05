import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_bot'
    
    # 1. 경로 설정 수정
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.sdf') # 월드 파일 경로

    # URDF 변환
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # gz_args에 world_file 경로를 전달합니다.
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. 로봇 소환
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.5'],
        output='screen'
    )

    # 4. Robot State Publisher (TF 트리 생성 핵심)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 5. ROS-Gazebo Bridge (토픽 경로 정밀 수정)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # gz topic -l 에서 확인된 실제 경로 사용
            '/world/empty/model/my_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/my_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/empty/model/my_bot/joint_state', '/joint_states'),
            ('/model/my_bot/tf', '/tf')
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. SLAM Toolbox 수정
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
            'scan_topic': '/scan',
            'mode': 'mapping',
            'transform_timeout': 0.5,
            'map_update_interval': 1.0,
        }]
    )

# 7. Static TF Publisher (Gazebo의 센서 프레임과 URDF의 프레임을 연결)
    node_static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        # 'laser_frame'이 부모, 'my_bot/base_link/lidar'가 자식임을 명시
        # 이렇게 하면 Gazebo에서 오는 데이터를 URDF의 laser_frame 위치로 인식합니다.
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'my_bot/base_link/lidar'],
        parameters=[{'use_sim_time': True}]
    )

# 8. Lifecycle Manager 추가 (SLAM 노드를 자동으로 활성화)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    )    

    delayed_spawn_entity = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    delayed_lifecycle_manager = TimerAction(
        period=2.0, 
        actions=[node_lifecycle_manager]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        bridge,
        slam_toolbox,
        node_static_tf_lidar,
        delayed_lifecycle_manager, # 지연 실행 적용
        delayed_spawn_entity
    ])