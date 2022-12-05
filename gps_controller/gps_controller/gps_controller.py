import rclpy 
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from osmnx import bearing, distance
import utm
from .generate_path import generate_path


class GpsController(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('gps_controller')
        
        # Adjustable Params
        refresh_hz = 5  # refresh rate of controller
        imu_msg = '/imu' 
        gps_msg = '/gps'
        error_topic_name = '/error'

        # Path Planner Params
        self.target_position = (32.867948, -117.203287)
        self.perimeter = 0.01  # Perimeter around target position to generate map
        self.mode = 'drive'  # Mode for map, acceptable values are 'drive', 'bike', 'walk'
        self.proxim_threshold = 0.5  # distance in m from waypoint to be considered complete

        # Vesc Steering Params
        self.steering_gain = 0.02  # gain value for steering algorithm
        self.max_right_steering = 1.0
        self.max_left_steering = 0.0
        self.straight_steering = 0.5

        # Create Subscribers and Publisher
        self.imu_subscriber = self.create_subscription(Imu, imu_msg, self.imu_callback, 10)
        self.gps_subscriber = self.create_subscription(PoseStamped, gps_msg, self.gps_callback, 10)
        self._error_publisher = self.create_publisher(Float32MultiArray, error_topic_name, 10)
        self.error_msg = Float32MultiArray()
        self.current_heading = 0.0
        self.current_position = (0, 0)
        self.previous_position = (0, 0)
        
        # Set Path variables
        self.target_index = 0
        self.target_final = 0
        self.current_waypoint_start = (0, 0)
        self.current_waypoint_finish = (0, 0)
        self.path = []
        self.running = False
        self.finished = False

        # Start Timer
        self.get_logger().info("Setup Complete")
        self.timer = self.create_timer(1.0 / refresh_hz, self.error_publisher)
        
    
    def error_publisher(self):
        """
        Main error publisher code. Generates path from current position to 
        target position, then starts navigating along the path.

        When robot gets close enough to next waypoint, iterate that until 
        we run out of waypoints. Calculate pid error values and send to the
        pid controller which can control the motor accordingly. 
        """
        self.get_logger().info("controlling")

        # If we're running, calculate the steering
        if self.running and not self.finished:
            # Calculate distance to goal waypoint
            distance_to_goal = self.distance_between_gps_points(self.current_position, self.current_waypoint_finish)
            
            self.get_logger().info(f"Distance: {distance_to_goal}")
            # If we're close enough to the waypoint, iterate waypoints.
            if distance_to_goal <= self.proxim_threshold:
                self.get_logger().info(f"Completed Waypoint {self.target_index}: ({self.target_position[0]} ,{self.target_position[1]})")
                self.target_index += 1

                # If we're done with the path, set finished to true and running to false
                if self.target_index == self.target_final:
                    self.get_logger().info(f"Completed all waypoints, idiling")
                    self.finished = True
                    self.running = False
                # Otherwise, increment the target position to the next pos in the path
                else:
                    self.current_waypoint_start = self.path[self.target_index]
                    self.current_waypoint_finish = self.path[self.target_index+1]
            
            # If we aren't close enough to waypoint, calculate pid
            else:
                cross_track_error, delta_theta = self.calculate_pid(distance_to_goal)
                msg: list[float] = [float(cross_track_error), float(delta_theta)]
                self.error_msg.data = msg
                self._error_publisher.publish(self.error_msg)

        # If we aren't finished and we're not running, calculate the path
        elif not self.finished and not self.running:
            if self.current_position[0] != 0:
                self.get_path()
                self.get_logger().info("Path Received!")
            else:
                self.get_logger().info("Waiting on GPS fix")

        # Otherwise, we're finished and not running so do nothing
        else:
            self.error_msg.data = [100000.0, 100000.0]
            self._error_publisher.publish(self.error_msg)


    def calculate_pid(self, distance_to_goal):
        """
        Function to calculate pid error values using two
        methods: Cross Track Error and Heading Error.

        References:
        (1) https://honors.libraries.psu.edu/files/final_submissions/3482
        (2) http://www.movable-type.co.uk/scripts/latlong.html
        """

        # Cross Track Error Formula 1 from reference (1)
        R = 6371e3
        d = self.distance_between_gps_points(self.current_waypoint_start, self.current_position) / R
        delta_1 = self.bearing_between_gps_points(self.current_waypoint_start, self.current_position) * (math.pi / 180)
        delta_2 = self.bearing_between_gps_points(self.current_waypoint_start, self.current_waypoint_finish) * (math.pi / 180)
        self.get_logger().info(f"D: {d}, delta1: {delta_1}, delta2: {delta_2}")
        cross_track_error_1 = math.asin(math.sin(d)*math.sin(delta_1-delta_2)) * R;

        # Cross Track Error Formula 2 from reference (2)
        start_x, start_y, _, _ = utm.from_latlon(self.current_waypoint_start[0], self.current_waypoint_start[1])
        finish_x, finish_y, _, _ = utm.from_latlon(self.current_waypoint_finish[0], self.current_waypoint_finish[1])
        current_x, current_y, _, _ = utm.from_latlon(self.current_position[0], self.current_position[1])
        a = start_y - finish_y
        b = finish_x - start_x
        c = start_x*finish_y - finish_x*start_y
        cross_track_error_2 = (-1)*((a*current_x) + (b*current_y) + c) / math.sqrt((a*a)+(b*b))

        # Heading Error
        bearing_to_goal = self.bearing_between_gps_points(self.current_position, self.current_waypoint_finish)
        current_bearing = self.current_heading

        # Another option is to calculate current bearing with previous position
        # current_bearing = self.bearing_between_gps_points(self.previous_position, self.current_position)
        
        delta_theta = current_bearing - bearing_to_goal 
        if delta_theta < -180:  # Change heading error to be on -180 to 180 scale
            delta_theta += 360
        elif delta_theta > 180:
            delta_theta -= 360
        delta_theta = delta_theta / 180.0  # normalize from -1 to 1
        
        # Average the two cross track calculations
        cross_track_error = ((cross_track_error_1 + cross_track_error_2) / 2.0) 

        # Optional: Scale cross track error by distance to point
        cross_track_error = cross_track_error / distance_to_goal

        self.get_logger().info(f"Delta Theta: {delta_theta}, Cross Track Average: {cross_track_error}")
        return cross_track_error, delta_theta


    def get_path(self):
        """
        Calls the generate path function using current position and target position to
        create a path of points for the robot to follow.

        Also sets the target_position as the first waypoint, and sets up the final
        target in the path array.
        """

        try:
            # Call generate path function to create a path from current to target position
            self.path = generate_path(self.current_position, self.target_position, self.perimeter, self.get_logger(), self.mode)
            self.get_logger().info(f"Current Position: {self.current_position}, Target Position: {self.target_position}")
            self.get_logger().info(f"Path: {self.path}")
            
            # Set first and final target positions of path
            self.current_waypoint_start = self.path[self.target_index]
            self.current_waypoint_finish = self.path[self.target_index+1]
            self.target_final = len(self.path) - 1
            self.running = True  # Starts the steering controls

        except Exception as e:
            self.get_logger().info(f"Couldn't parse path, {e}")

            # If we can't generate a path, set finished to True and running to False
            self.running = False
            self.finished = True
        

    def imu_callback(self, imu_msg: Imu):
        """
        Function that's called when IMU msg is received, 
        only storing heading
        """
        try:
            self.current_heading = float(imu_msg.orientation.z)

        except Exception as e:
            self.get_logger().info(f"{e}")


    def gps_callback(self, gps_msg: PoseStamped):
        """
        Function that's called when GPS msg is received,
        storing current lat lon and previous lat lon
        """
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


    def distance_between_gps_points(self, pos1, pos2):
        """
        Reference for formulas:
        https://www.movable-type.co.uk/scripts/latlong.html
        """
        lat1 = pos1[0]
        lon1 = pos1[1]
        lat2 = pos2[0]
        lon2 = pos2[1]
        d = distance.great_circle_vec(lat1, lon1, lat2, lon2)
        return d


    def bearing_between_gps_points(self, pos1, pos2):
        """
        Reference for formulas:
        https://www.movable-type.co.uk/scripts/latlong.html
        """
        lat1 = pos1[0]
        lon1 = pos1[1]
        lat2 = pos2[0]
        lon2 = pos2[1]
        b = bearing.calculate_bearing(lat1, lon1, lat2, lon2)
        return b


def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    gps_controller = GpsController() # declare the node constructor
    rclpy.spin(gps_controller) # pause the program execution, waits for a request to kill the node (ctrl+c)
    gps_controller.shutdown_vesc()
    gps_controller.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()