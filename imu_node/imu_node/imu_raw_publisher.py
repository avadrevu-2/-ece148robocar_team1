import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
 
 
class IMURawPublisher(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('imu_raw_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', 1)
        # define the timer period 
        hz = 52.0
        timer_period = 1.0 / hz 
        # open serial port
        self.ser = serial.Serial('/dev/ttyACM1', 9600)
        # IMU message
        self.imu_msg = Imu()
        self.get_logger().info("Setup Complete")
        self.timer = self.create_timer(timer_period, self.imu_pub_callback)


    def imu_pub_callback(self):
        try:
            # Read from Serial
            msg = self.ser.readline().decode("utf-8")
            (_, ax, ay, az, gx, gy, gz, roll, pitch, heading) = msg.split(',')

            # Add linear acceleration and angular velocity values
            self.imu_msg.linear_acceleration.x = float(ax)
            self.imu_msg.linear_acceleration.y = float(ay)
            self.imu_msg.linear_acceleration.z = float(az)
            self.imu_msg.angular_velocity.x = float(gx)
            self.imu_msg.angular_velocity.y = float(gy)
            self.imu_msg.angular_velocity.z = float(gz)

            # Add Roll Pitch Heading to orientation
            self.imu_msg.orientation.x = float(roll)
            self.imu_msg.orientation.y = float(pitch)
            self.imu_msg.orientation.z = float(heading)
            self.imu_msg.orientation.w = 0.0

            # add timestamp and publish
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.imu_msg)
            self.get_logger().info(f"Heading: {heading}")

        except Exception as e:
            self.get_logger().info(f"{e}")



def main(args=None):
    rclpy.init(args=args) # initialize the ROS communication
    imu_publisher = IMURawPublisher() # declare the node constructor
    rclpy.spin(imu_publisher) # pause the program execution, waits for a request to kill the node (ctrl+c)
    imu_publisher.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # shutdown the ROS communication
 
if __name__ == '__main__':
    main()