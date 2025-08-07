from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='create2_hardware',
      executable='create2_hardware_interface',
      name='create2_hardware_node'
    )
  ])
