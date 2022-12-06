import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
import yaml


node_list_input_path = '/home/projects/ros2_ws/src/ucsd_robocar_hub2/gps_controller/config/node_config.yaml'
node_packages_path = '/home/projects/ros2_ws/src/ucsd_robocar_hub2/gps_controller/config/node_pkg_locations.yaml'

def update_parameters(node_list_input_path):
        with open(node_list_input_path, "r") as file:
            sensor_inputs = yaml.load(file, Loader=yaml.FullLoader)
            selected_inputs = {}
            for sensor in sensor_inputs:
                value = sensor_inputs[sensor]
                if value==1:
                    selected_inputs[sensor] = value
            return selected_inputs


def update_packages(node_packages_path):
    with open(node_packages_path, "r") as file:
            packages_dict = yaml.load(file, Loader=yaml.FullLoader)
            car_inputs_dict = update_parameters(node_list_input_path)
            my_packages = {}
            for key in packages_dict:
                value = packages_dict[key]
                if key in car_inputs_dict:
                    my_packages[key] = value
            return my_packages


def generate_a_launch_description(some_package, some_launch):
    return LaunchDescription([
    DeclareLaunchArgument(
            'topic_name',
            default_value = 'default_value',
            description = 'REQUIRED argument for new topic name (can use original topic name if needed)'
        ),
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(some_package),
                    'launch',
                    some_launch)
            ),
        ),
    ])


def generate_launch_description():
    my_packages_dict = update_packages(node_packages_path)
    ld = LaunchDescription()
    for key in my_packages_dict:
        pkg_name = my_packages_dict[key][0]
        launch_name = my_packages_dict[key][1]
        try:
            ld.add_action(generate_a_launch_description(pkg_name, launch_name))
        except:
            print(f"Exception on {key}")
            pass
        print(f"Trying to start: {launch_name} from {pkg_name}")
    
    gps_controller = Node(
           package='gps_controller',
           executable='gps_controller')
 
    ld.add_action(gps_controller)

    return ld
