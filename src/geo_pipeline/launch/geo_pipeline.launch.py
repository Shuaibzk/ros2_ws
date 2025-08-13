from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='geo_pipeline',
            executable='geo_compute_node',
            name='geo_compute',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='geo_pipeline',
            executable='result_publisher_node',
            name='result_publisher',
            output='screen',
            emulate_tty=True
        ),
    ])

