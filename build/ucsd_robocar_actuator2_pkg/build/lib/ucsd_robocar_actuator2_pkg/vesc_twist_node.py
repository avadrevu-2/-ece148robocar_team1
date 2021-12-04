import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .vesc_submodule.vesc_client import VESC_

NODE_NAME = 'vesc_twist_node'
TOPIC_NAME = '/cmd_vel'

v = VESC_()

class VescTwist(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rpm_subscriber = self.create_subscription(Twist, TOPIC_NAME, self.callback, 10)

        # Default actuator values
        self.default_rpm_value = int(5000)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_rpm', self.default_rpm_value)
            ])
        self.max_rpm = self.get_parameter('max_rpm').value

    def callback(self, msg):
        # Steering map from [-1,1] --> [0,1]  
        vesc_min_limit = 0
        vesc_max_limit = 1
        data_min_limit = -1
        data_max_limit = 1
        steering_angle = float(-0.1 + ((msg.angular.z-data_min_limit)*(vesc_max_limit - vesc_min_limit))/(data_max_limit-data_min_limit))
        
        # RPM map from [-1,1] --> [-max_rpm,max_rpm]
        rpm = int(self.max_rpm * msg.linear.x)

        v.send_rpm(rpm)
        v.send_servo_angle(steering_angle)


def main(args=None):
    rclpy.init(args=args)
    vesc_twist = VescTwist()
    rclpy.spin(vesc_twist)
    vesc_twist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
