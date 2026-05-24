from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
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
        
        # 5. Нода имитации потолочного детектора (Fake tag publisher)
        Node(
            package='fake_tag_publisher',
            executable='fake_tag_node',
            name='fake_tag_node',
            output='screen'
        ),
        
        # 6. Нода локализации и слияния данных (Localization node)
        Node(
            package='fake_tag_publisher',
            executable='localization_node',
            name='localization_node',
            output='screen'
        ),
    ])
