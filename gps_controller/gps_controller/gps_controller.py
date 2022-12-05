import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import json
import math

from pyvesc import VESC

class GpsController(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('gps_controller')
        
        # Adjustable Params
        vesc_port = "/dev/ttyACM0"
        refresh_hz = 5
        imu_msg = '/imu'
        gps_msg = '/gps'
        self.max_right_steering = 1.0
        self.max_left_steering = 0.0
        self.straight_steering = 0.5
        self.max_rpm_value = 20000
        self.path_location = "/home/projects/ros2_ws/src/ucsd_robocar_hub2/gps_controller/paths/test_path.json"
        self.steering_gain = 0.02
        self.proxim_threshold = 1.0  # distance in m from waypoint to be considered complete

        # Create Vesc Object
        try:
            self.vesc = VESC(serial_port=vesc_port, has_sensor=False, start_heartbeat=True, baudrate=115200)
            self.vesc.set_rpm(0)
            self.get_logger().info("Succesfully Connected to VESC!")

        except Exception as e:
            self.get_logger().info(f"Could not connect to VESC, {e}")
            exit()

        # Create Subscribers
        self.imu_subscriber = self.create_subscription(Imu, imu_msg, self.imu_callback, 10)
        self.gps_subscriber = self.create_subscription(PoseStamped, gps_msg, self.gps_callback, 10)
        self.current_heading = 0.0
        self.current_position = (0, 0)
        self.previous_position = (0, 0)
        
        # Get Path
        self.path = []
        try:
            with open(self.path_location) as file:
                data = json.load(file)
                for value in data['coordinates']:
                    self.path.append((value[1], value[0]))
                self.get_logger().info(f"Path: {self.path}")

        except Exception as e:
            self.get_logger().info(f"Couldn't parse path, {e}")

        # Set first and final target positions of path
        self.target_position = self.path[0]
        self.target_index = 0
        self.target_final = len(self.path)
        self.running = True
        self.get_logger().info("Setup Complete")
        self.timer = self.create_timer(1.0 / refresh_hz, self.controller)
    

    def imu_callback(self, imu_msg: Imu):
        try:
            self.current_heading = float(imu_msg.orientation.z)

        except Exception as e:
            self.get_logger().info(f"{e}")


    def gps_callback(self, gps_msg: PoseStamped):
        try:
            # Keep track of previous lat lon
            prev_lat = self.current_position[0]
            prev_lon = self.current_position[1]
            # Get current lat lon from Gps msg
            curr_lat = gps_msg.pose.position.x
            curr_lon = gps_msg.pose.position.y
            # Set the self variables
            self.current_position = (curr_lat, curr_lon)
            self.previous_position = (prev_lat, prev_lon)
        
        except Exception as e:
            self.get_logger().info(f"{e}")


    def controller(self):
        self.get_logger().info("controlling")
        if self.running:
            # Calculate distance to goal waypoint
            distance_to_goal = self.distance_between_gps_points(self.current_position, self.target_position)
            self.get_logger().info(f"Distance: {distance_to_goal}")

            # If we're close enough to the waypoint, iterate waypoints.
            if distance_to_goal <= self.proxim_threshold:
                self.get_logger().info(f"Completed Waypoint {self.target_index}: ({self.target_position[0]} ,{self.target_position[1]})")
                self.target_index += 1

                # If we're done with the path, set running to false
                if self.target_index == self.target_final:
                    self.get_logger().info(f"Completed all waypoints, idiling")
                    self.running = False
                # Otherwise, increment the target position to the next pos in the path
                else:
                    self.target_position = self.path[self.target_index]
            
            # If we aren't close enough to waypoint, calculate steering
            else:
                command = self.calculate_steering()

                # Set the VESC RPM and Steering Angle
                self.vesc.set_rpm(int(self.max_rpm_value * 0.5))
                self.vesc.set_servo(float(command))
        else:
            self.vesc.set_rpm(0)
    
    
    def calculate_steering(self):
        # Calculate bearing differential
        bearing_to_goal = self.bearing_between_gps_points(self.current_position, self.target_position)
        bearing_to_previous = self.bearing_between_gps_points(self.previous_position, self.current_position)
        
        # Option 1: Using IMU reading for current bearing
        current_bearing = self.current_heading
        
        # Option 2: Using Previous GPS reading for current bearing
        # current_bearing = bearing_to_previous

        # Calculate steering command using gain factor
        steering_command = self.steering_gain * (bearing_to_goal - current_bearing)

        # Clamp the steering command to max right and max left
        if steering_command > self.max_right_steering:
            steering_command = self.max_right_steering
        elif steering_command < self.max_left_steering:
            steering_command = self.max_left_steering
        
        self.get_logger().info(f"Headings: To Goal: {bearing_to_goal}, Current: {current_bearing}, To Previous: {bearing_to_previous}, Command: {steering_command}")
        return steering_command


    def distance_between_gps_points(self, pos1, pos2):
        """
        Reference for formulas:
        https://www.movable-type.co.uk/scripts/latlong.html
        """
        # Setup variables (converting to radians and calculating deltas)
        lat1 = pos1[0]
        lon1 = pos1[1]
        lat2 = pos2[0]
        lon2 = pos2[1]
        radius = 6371e3
        scaler = math.pi/180
        phi1 = lat1 * scaler
        phi2 = lat2 * scaler
        delta_phi = (lat2 - lat1) * scaler
        delta_lambda = (lon2 - lon1) * scaler

        # Calculate Distance using Haversine formula
        a = (math.sin(delta_phi/2) * math.sin(delta_phi/2)) + \
            (math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2) * math.sin(delta_lambda/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = radius * c
        return d


    def bearing_between_gps_points(self, pos1, pos2):
        """
        Reference for formulas:
        https://www.movable-type.co.uk/scripts/latlong.html
        """
        # Setup variables (converting to radians)
        lat1 = pos1[0]
        lon1 = pos1[1]
        lat2 = pos2[0]
        lon2 = pos2[1]
        scaler = math.pi / 180
        phi1= lat1 * scaler
        phi2 = lat2 * scaler
        lambda1 = lon1 * scaler
        lambda2 = lon2 * scaler

        # Calculate Angle
        y = math.sin(lambda2 - lambda1) * math.cos(phi2)
        x = (math.cos(phi1) * math.sin(phi2)) - (math.sin(phi1) * math.cos(phi2) * math.cos(lambda2 - lambda1))
        theta = math.atan2(y, x)

        # Adjust Angle to be on 0 to 360 degree scale
        bearing = ((theta*180/math.pi) + 360) % 360
        return bearing


    def destroy_node(self):
        self.vesc.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    gps_controller = GpsController() # declare the node constructor
    rclpy.spin(gps_controller) # pause the program execution, waits for a request to kill the node (ctrl+c)
    gps_controller.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()