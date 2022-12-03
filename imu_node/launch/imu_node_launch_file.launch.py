import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
   package_name = 'imu_node'
 
   ld = LaunchDescription()
 
   pub_counter_node = Node(
           package=package_name,
           executable='imu_raw_publisher',
           output='screen')
 
   ld.add_action(pub_counter_node)
   return ld