from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('contact_point_estimation').find('contact_point_estimation')

    # YAML parameter files
    cpe_yaml = os.path.join(pkg_share, 'config', 'contact_point_estimator.yaml')
    sne_yaml = os.path.join(pkg_share, 'config', 'surface_normal_estimator.yaml')

    # Node definition
    contact_point_node = Node(
        package='contact_point_estimation',
        executable='contact_point_estimation_node',
        name='contact_point_estimation',
        output='screen',
        parameters=[cpe_yaml, sne_yaml]
    )

    return LaunchDescription([
        contact_point_node
    ])
