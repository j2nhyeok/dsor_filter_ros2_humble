import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dsor_filter',
            executable='example_node', 
            name='example',
            output='screen',
            remappings=[
                ('cloud_in', '/points_raw'),
                ('filtered_cloud', 'dsor_cloud')
            ],
            parameters=[{
                // Adjust the DSOR parameter values as needed. 
                'k': 5,
                'std': 0.01,
                'range_mul': 0.05
            }]
        )
    ])
