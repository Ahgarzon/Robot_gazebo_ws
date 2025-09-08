from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cloud_rgb = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        remappings=[
            # lo que el nodo espera (izq)  →  lo que realmente publica Gazebo (dcha)
            ('depth/image_rect',       '/depth_camera/depth/image_raw'),
            ('rgb/image_rect_color',   '/rgb_camera/image_raw'),
            ('camera_info',            '/rgb_camera/camera_info'),
            ('points',                 '/depth_camera/points_rgb')
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',
                   '${find modelo_robot}/share/modelo_robot/rviz_depth_cloud.rviz']
    )

    return LaunchDescription([cloud_rgb, rviz])
