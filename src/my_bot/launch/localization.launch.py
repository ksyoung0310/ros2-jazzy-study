import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_name = 'my_bot'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 경로 설정
    # 저장한 맵 파일의 경로를 지정하세요.
    map_file_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Map Server 노드
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'yaml_filename': map_file_path}]
    )

    # 3. AMCL 노드 설정
    # 로봇의 특성(Mecanum/Diff)과 토픽명을 설정합니다.
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'scan_topic': '/scan',
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel', # 제공된 URDF가 DiffDrive이므로
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.yaw': 0.0
        }]
    )

    # 4. Lifecycle Manager (중요: AMCL과 Map Server를 활성화 상태로 전환)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        node_map_server,
        node_amcl,
        node_lifecycle_manager
    ])