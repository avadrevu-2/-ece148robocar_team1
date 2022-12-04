import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from transformations import quaternion_about_axis
import serial
 
 
class IMURawPublisher(Node):
    def __init__(self):
        # call super() in the constructor in order to initialize the Node object with node name as only parameter
        super().__init__('imu_raw_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', 1)
        timer_period = 0.2 # define the timer period 
        
        # open serial port
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        # GPS message
        self.imu_msg = Imu()

        self.get_logger().info("Setup Complete")

        self.timer = self.create_timer(timer_period, self.talker_callback)



    def talker_callback(self):
        try:
            # Read from Serial
            msg = self.ser.readline().decode("utf-8")
            self.get_logger().info(msg)
            (_, ax, ay, az, gx, gy, gz) = msg.split(',')

            # Add linear acceleration and angular velocity values
            self.imu_msg.linear_acceleration.x = float(ax)
            self.imu_msg.linear_acceleration.y = float(ay)
            self.imu_msg.linear_acceleration.z = float(az)
            self.imu_msg.angular_velocity.x = float(gx)
            self.imu_msg.angular_velocity.y = float(gy)
            self.imu_msg.angular_velocity.z = float(gz)

            #Calculate Orientation Quaternion
            """
            Quaternion calculation code taken from:
            https://github.com/bandasaikrishna/orientations_from_IMU_MPU_6050/blob/main/mpu_6050_driver/scripts/imu_node.py
            """
            accel = self.imu_msg.linear_acceleration.x, \
                self.imu_msg.linear_acceleration.y, \
                self.imu_msg.linear_acceleration.z
            ref = np.array([0, 0, 1])
            acceln = accel / np.linalg.norm(accel)
            try:
                axis = np.cross(acceln, ref)
            except ValueError:
                self.get_logger().info("Error on Cross Product")
            angle = np.arccos(np.dot(acceln, ref))
            orientation = quaternion_about_axis(angle, axis)

            # Add Orientation Quat to message
            o = self.imu_msg.orientation
            o.x, o.y, o.z, o.w = orientation

            # add timestamp and publish
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.imu_msg)

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