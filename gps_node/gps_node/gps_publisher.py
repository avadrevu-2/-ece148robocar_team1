import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from dms2dec.dms_convert import dms2dec
import pynmea2
import serial
import math
 

class GpsPublisher(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/GpsFix', 1)
        timer_period = 0.2 # define the timer period 
        
        # open serial port
        self.ser = serial.Serial('/dev/ttyUSB2', 115200)
        # GPS message
        self.gps_msg = NavSatFix()

        self.get_logger().info("Setup Complete")
        self.timer = self.create_timer(timer_period, self.talker_callback)


    def talker_callback(self):
        try:
            msg = self.ser.readline().decode("utf-8")
            if msg[0:6] in ["$GPGGA", "$GNRMC"]:
                # Parse message
                parsed = pynmea2.parse(msg)
                # convert to degrees                
                raw_lat = float(parsed.lat)
                raw_lon = float(parsed.lon)
                lat = f"{int(raw_lat//100)}°{math.trunc(raw_lat%100)}\'{(raw_lat%1)*60}\"{parsed.lat_dir}"
                lon = f"{int(raw_lon//100)}°{math.trunc(raw_lon%100)}\'{(raw_lon%1)*60}\"{parsed.lon_dir}"
                self.get_logger().info(f"{lat}, {lon}, {parsed.timestamp}")
                # add to message
                self.gps_msg.latitude = dms2dec(lat)
                self.gps_msg.longitude = dms2dec(lon)
                self.gps_msg.status.status = 0
                self.gps_msg.status.service = 1
                self.gps_msg.header.stamp = self.get_clock().now().to_msg()
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