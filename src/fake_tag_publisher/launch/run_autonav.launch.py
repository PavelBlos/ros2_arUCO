from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Аргументы запуска
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 stepper driver connection'
    )

    calibration_path_arg = DeclareLaunchArgument(
        'calibration_path',
        default_value='config/camera_info.yaml',
        description='Path to the camera calibration parameters'
    )
    
    dictionary_arg = DeclareLaunchArgument(
        'aruco_dictionary',
        default_value='DICT_4X4_100',
        description='ArUco dictionary name'
    )

    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='0', # 0 = /dev/video0
        description='Live camera index or path'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Run in simulation mode (SIL)'
    )

    return LaunchDescription([
        port_arg,
        calibration_path_arg,
        dictionary_arg,
        video_path_arg,
        sim_arg,
        
        # 1. Статический TF: base_link -> camera_link (камера на роботе смотрит строго вверх)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '-1.570796', # -90 градусов по pitch
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ]
        ),
        
        # 2. Мост связи с ESP32 (команды движения и колесная одометрия)
        Node(
            package='fake_tag_publisher',
            executable='esp32_bridge',
            name='esp32_bridge',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': 115200,
                'wheel_radius': 0.030,
                'base_radius': 0.122,
                'steps_per_rev': 3200, # 200 шагов * 16 микрошагов
                'sim': LaunchConfiguration('sim'),
            }],
            output='screen'
        ),
        
        # 3. Детектор ArUco меток с живой камеры
        Node(
            package='fake_tag_publisher',
            executable='video_tag_detector',
            name='video_tag_detector',
            parameters=[{
                'video_path': LaunchConfiguration('video_path'),
                'calibration_path': LaunchConfiguration('calibration_path'),
                'aruco_dictionary': LaunchConfiguration('aruco_dictionary'),
                'loop_video': False, # Не закрываемся при работе с камерой
                'marker_length': 0.15,
                'detection_rate': 30.0
            }],
            output='screen'
        ),
        
        # 4. Основная нода локализации (с Web UI и комплементарным фильтром слияния)
        Node(
            package='fake_tag_publisher',
            executable='localization_node',
            name='localization_node',
            parameters=[{
                'filter_alpha': 0.20 # Вес доверяемости ArUco при слиянии
            }],
            output='screen'
        ),

        # 5. Нода автоматического следования по траектории (Pure Pursuit)
        Node(
            package='fake_tag_publisher',
            executable='path_follower',
            name='path_follower',
            parameters=[{
                'look_ahead_distance': 0.001, # м
                'max_linear_velocity': 0.18, # м/с
                'max_angular_velocity': 0.8, # рад/с
                'kp_linear': 0.8,
                'kp_angular': 1.5,
                'goal_tolerance': 0.01,      # м (1 см)
                'yaw_deadzone_dist': 0.02,    # м (2 см)
                'waypoint_tolerance': 0.01,   # м (точность точек)
            }],
            output='screen'
        ),
    ])
