import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int32
from custom_interfaces.msg import Gps
from dms2dec.dms_convert import dms2dec
import pynmea2
import serial
import datetime
import math
 
class GpsPublisher(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(Gps, '/GpsFix', 1)
        timer_period = 0.5 # define the timer period 
        
        # open serial port
        self.ser = serial.Serial('/dev/ttyUSB1', 115200)
        # GPS message
        self.gps_msg = Gps()

        self.get_logger().info("Setup Complete")

        self.timer = self.create_timer(timer_period, self.talker_callback)



    def talker_callback(self):
        try:
            msg = self.ser.readline().decode("utf-8")
            self.get_logger().info(msg)
            if msg[0:6] in ["$GPGGA", "$GNRMC"]:
                parsed = pynmea2.parse(msg)
                raw_lat = float(parsed.lat)
                raw_lon = float(parsed.lon)
                lat = f"{int(raw_lat//100)}°{math.trunc(raw_lat%100)}\'{(raw_lat%1)*60}\"{parsed.lat_dir}"
                lon = f"{int(raw_lon//100)}°{math.trunc(raw_lon%100)}\'{(raw_lon%1)*60}\"{parsed.lon_dir}"

                self.get_logger().info(f"{lat}, {lon}, {parsed.timestamp}")
                self.gps_msg.latitude = dms2dec(lat)
                self.gps_msg.longitude = dms2dec(lon)
                self.gps_msg.time = datetime.datetime.now().timestamp()
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