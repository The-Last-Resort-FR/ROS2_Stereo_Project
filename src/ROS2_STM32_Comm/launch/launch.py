from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm_comm',
            executable='stm_comm',
            name='stm_comm',
            parameters=[{'bus_name': '/dev/ttyACM2'}],
        )
    ])