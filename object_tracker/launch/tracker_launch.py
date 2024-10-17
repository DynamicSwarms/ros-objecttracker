import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('object_tracker'),
                          'launch',
                          'tracker_config.yaml')
    node = Node(
            package='object_tracker',
            #namespace='object_tracker',
            executable='tracker',
            name='tracker',
            parameters = [config]
        )
    
    pcl_test = Node(
        package='test_pcl_publisher',
        executable='pcl_publisher',
        name='pcl_pub'
    )
    

    ld.add_action(node)
    ld.add_action(pcl_test)
    return ld
    