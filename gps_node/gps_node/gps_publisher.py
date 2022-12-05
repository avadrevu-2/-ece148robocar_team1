import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pynmea2
import serial


class GpsPublisher(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/gps', 1)
        hz = 20
        timer_period = 1.0 / hz
        # open serial port
        self.ser = serial.Serial('/dev/ttyUSB1', 115200)
        # GPS message
        self.gps_msg = PoseStamped()
        self.latitude = 0
        self.longitude = 0
        # Create Timer for publisher callback
        self.get_logger().info("Setup Complete")
        self.timer = self.create_timer(timer_period, self.gps_pub_callback)

    def parse_msg(self, msg):
        """
        Function to parse msg from gps sensor, convert from
        degrees to decimal, then covert to utm. 
        Stores the output in self.position_x and self.position_y
        """
        try:     
            # parse out degrees    
            parsed = pynmea2.parse(msg)
            if parsed.latitude != 0 and parsed.longitude != 0:
                self.latitude = parsed.latitude
                self.longitude = parsed.longitude
                self.get_logger().info(f"{self.latitude}, {self.longitude}")
                return True

        except Exception as e:
            self.get_logger().info(f"{e}")
        
        return False


    def gps_pub_callback(self):
        try:
            msg = self.ser.readline().decode("utf-8")
            # Check that msg type is GNRMC
            if msg[0:6] in ["$GNRMC", "$GPGGA"]:
                # Parse message, if it fails don't do anything
                if not self.parse_msg(msg):
                    return
                # Add data
                self.gps_msg.pose.position.x = float(self.latitude)
                self.gps_msg.pose.position.y = float(self.longitude)
                self.gps_msg.pose.position.z = 0.0
                self.gps_msg.header.stamp = self.get_clock().now().to_msg()
                # publish message
                self.publisher_.publish(self.gps_msg)

        except Exception as e:
            self.get_logger().info(f"{e}")



def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    gps_publisher = GpsPublisher() # declare the node constructor
    rclpy.spin(gps_publisher) # pause the program execution, waits for a request to kill the node (ctrl+c)
    gps_publisher.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()