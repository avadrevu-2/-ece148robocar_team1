import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
   package_name = 'gps_controller'
 
   ld = LaunchDescription()
 
   gps_controller = Node(
           package=package_name,
           executable='gps_controller',
           output='screen')
 
   ld.add_action(gps_controller)
   return ld