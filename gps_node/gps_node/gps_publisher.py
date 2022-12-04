import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from dms2dec.dms_convert import dms2dec
import pynmea2
import serial
import math
import utm
from time import sleep

class GpsPublisher(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(Pose, '/amcl_pose', 1)
        hz = 8
        timer_period = 1.0 / hz
        
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

        # open serial port
        self.ser = serial.Serial('/dev/ttyUSB2', 115200)
        x_lat = 32.86888560333333
        y_lon = -117.20253661500001
        x, y, _, _ = utm.from_latlon(x_lat, y_lon)
        self.x_0 = x
        self.y_0 = y


        # GPS message
        self.gps_msg = Pose()

        self.get_logger().info("Setup Complete")
        self.timer = self.create_timer(timer_period, self.talker_callback)


    def imu_callback(self, imu_data):
        try:
            self.orientation_x = float(imu_data.orientation.x)
            self.orientation_y = float(imu_data.orientation.y)
            self.orientation_z = float(imu_data.orientation.z)
            self.orientation_w = float(imu_data.orientation.w)
        except Exception as e:
            self.get_logger().info(f"{e}")
            pass


    def parse_msg(self, msg):
        parsed = pynmea2.parse(msg)
        # convert to degrees                
        raw_lat = float(parsed.lat)
        raw_lon = float(parsed.lon)
        lat = f"{int(raw_lat//100)}°{math.trunc(raw_lat%100)}\'{(raw_lat%1)*60}\"{parsed.lat_dir}"
        lon = f"{int(raw_lon//100)}°{math.trunc(raw_lon%100)}\'{(raw_lon%1)*60}\"{parsed.lon_dir}"
        self.get_logger().info(f"{lat}, {lon}, {parsed.timestamp}")
        # add to message

        lat = dms2dec(lat)
        lon = dms2dec(lon)
        x, y, _, _ = utm.from_latlon(lat, lon)
        return x, y


    def talker_callback(self):
        try:
            msg = self.ser.readline().decode("utf-8")
            if msg[0:6] in ["$GPGGA", "$GNRMC"]:
                # Parse message
                x, y = self.parse_msg(msg)
                self.gps_msg.position.x = x - self.x_0
                self.gps_msg.position.y = y - self.y_0
                # self.gps_msg.position.x = x
                # self.gps_msg.position.y = y
                self.gps_msg.position.z = 0.0
                self.gps_msg.orientation.x = self.orientation_x
                self.gps_msg.orientation.y = self.orientation_y
                self.gps_msg.orientation.z = self.orientation_z
                self.gps_msg.orientation.w = self.orientation_w
             
                # publish message
                self.publisher_.publish(self.gps_msg)


        except Exception as e:
            self.get_logger().info(f"{e}")



def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    simple_publisher = GpsPublisher() # declare the node constructor
    rclpy.spin(simple_publisher) # pause the program execution, waits for a request to kill the node (ctrl+c)
    simple_publisher.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()