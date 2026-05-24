from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Объявляем аргументы запуска для удобной настройки путей
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='config/robot_drive.mp4',
        description='Absolute or relative path to the pre-recorded robot drive video'
    )
    
    calibration_path_arg = DeclareLaunchArgument(
        'calibration_path',
        default_value='config/camera_info.yaml',
        description='Path to the camera calibration parameters'
    )

    return LaunchDescription([
        video_path_arg,
        calibration_path_arg,
        
        # 1. Статический TF: base_link -> camera_link (камера смотрит вертикально ВВЕРХ, pitch = -90 градусов)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '-1.570796',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        ),
        
        # 2. Статический TF: map -> tag_1 (потолок, z = 2.5м, смотрит вниз, roll = 180 градусов)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_tag_1',
            arguments=[
                '--x', '2.0',
                '--y', '1.0',
                '--z', '2.5',
                '--roll', '3.1415926',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'tag_1'
            ]
        ),

        # 3. Статический TF: map -> tag_2 (потолок, z = 2.5м, смотрит вниз, roll = 180 градусов)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_tag_2',
            arguments=[
                '--x', '-1.0',
                '--y', '2.0',
                '--z', '2.5',
                '--roll', '3.1415926',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'tag_2'
            ]
        ),

        # 4. Статический TF: map -> tag_3 (потолок, z = 2.5м, смотрит вниз, roll = 180 градусов)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_tag_3',
            arguments=[
                '--x', '-2.0',
                '--y', '-1.0',
                '--z', '2.5',
                '--roll', '3.1415926',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'tag_3'
            ]
        ),
        
        # 5. Нода детектирования ArUco-маркеров по видеофайлу
        Node(
            package='fake_tag_publisher',
            executable='video_tag_detector',
            name='video_tag_detector',
            parameters=[{
                'video_path': LaunchConfiguration('video_path'),
                'calibration_path': LaunchConfiguration('calibration_path'),
                'marker_length': 0.15,
                'detection_rate': 10.0
            }],
            output='screen'
        ),
        
        # 6. Нода локализации и слияния (остается без изменений!)
        Node(
            package='fake_tag_publisher',
            executable='localization_node',
            name='localization_node',
            output='screen'
        ),
    ])
