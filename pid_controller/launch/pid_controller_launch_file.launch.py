import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
   package_name = 'pid_controller'
 
   ld = LaunchDescription()
 
   pid_node = Node(
           package=package_name,
           executable='pid_controller',
           output='screen')
 
   ld.add_action(pid_node)
   return ld