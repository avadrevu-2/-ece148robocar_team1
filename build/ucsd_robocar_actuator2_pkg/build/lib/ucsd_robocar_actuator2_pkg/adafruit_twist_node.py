import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .vesc_submodule.vesc_client import VESC_

NODE_NAME = 'adafruit_twist_node'
TOPIC_NAME = '/cmd_vel'

class AdafruitTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rpm_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.callback, 10)
        self.kit = ServoKit(channels=16)

    def callback(self, msg):
        # Steering map from [-1,1] --> [max_left,max_right] # to do: implement into calibration
        data_min_limit = -1
        data_max_limit = 1
        adafruit_min_limit = -1 # These will be rosparams eventually... : max_left
        adafruit_max_limit = 1  # These will be rosparams eventually... : max_right
        steering_angle = float(-0.1 + ((msg.angular.z-data_min_limit)*(vesc_max_limit - vesc_min_limit))/(data_max_limit-data_min_limit))

        # Send values to adafruit board
        self.kit.servo[1].angle = 90 * (1 + steering_angle)
        self.kit.continuous_servo[2].throttle = msg.linear.x


def main(args=None):
    rclpy.init(args=args)
    adafruit_twist = AdafruitTwist()
    rclpy.spin(adafruit_twist)
    adafruit_twist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
