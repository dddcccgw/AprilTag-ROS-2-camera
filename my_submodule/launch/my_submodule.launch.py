from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    """Generate launch description for my_submodule."""
    
    package_dir = FindPackageShare(package='my_submodule')
    config_dir = PathJoinSubstitution([package_dir, 'config'])
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add my_submodule node
    my_submodule_node = Node(
        package='my_submodule',
        executable='my_submodule_node',
        name='my_submodule',
        output='screen',
        parameters=[
            os.path.join(str(config_dir), 'my_submodule.yaml')
        ],
        remappings=[
            ('image_raw', 'camera/image_raw'),
            ('output', 'my_submodule/output'),
        ]
    )
    
    ld.add_action(my_submodule_node)
    
    return ld
