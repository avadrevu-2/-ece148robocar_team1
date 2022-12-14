import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
   package_name = 'gps_node'
 
   ld = LaunchDescription()
 
   gps_node = Node(
           package=package_name,
           executable='gps_publisher',
           output='screen')
 
   ld.add_action(gps_node)
   return ld