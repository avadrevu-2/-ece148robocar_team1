import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Int32MultiArray
import time
import os

NODE_NAME = 'lane_guidance_node'
CENTROID_TOPIC_NAME = '/centroid'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'


class PathPlanner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.steering_publisher = self.create_publisher(Float32, STEERING_TOPIC_NAME, 10)
        self.throttle_publisher = self.create_publisher(Float32, THROTTLE_TOPIC_NAME, 10)
        self.steering_value = Float32()
        self.throttle_value = Float32()
        self.centroid_subscriber = self.create_subscription(Float32, CENTROID_TOPIC_NAME, self.controller, 10)
        self.centroid_subscriber

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_sensitivity', 1),
                ('no_error_throttle', 1),
                ('error_throttle', 1),
                ('error_threshold', 1),
                ('zero_throttle',1)
            ])
        self.steering_sensitivity = self.get_parameter('steering_sensitivity').value
        self.no_error_throttle = self.get_parameter('no_error_throttle').value
        self.error_throttle = self.get_parameter('error_throttle').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.get_logger().info(
            f'\nsteering_sensitivity: {self.steering_sensitivity}'
            f'\nno_error_throttle: {self.no_error_throttle}'
            f'\nerror_throttle: {self.error_throttle}'
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_throttle: {self.zero_throttle}'
        )

    def controller(self, data):
        try:
            kp = self.steering_sensitivity
            error_x = data.data
            self.get_logger().info(f"{error_x}")
            if error_x <= self.error_threshold:
                throttle_float = self.no_error_throttle
            else:
                throttle_float = self.error_throttle
            steering_float = float(kp * error_x)
            if steering_float < -1.0:
                steering_float = -1.0
            elif steering_float > 1.0:
                steering_float = 1.0
            else:
                pass
            self.steering_float.data = steering_float
            self.throttle_float.data = throttle_float
            self.steering_publisher.publish(self.steering_float)
            self.throttle_publisher.publish(self.throttle_float)
        except KeyboardInterrupt:
            self.throttle_publisher.data = self.zero_throttle
            self.throttle_publisher.publish(self.throttle_float)


def main(args=None):
    rclpy.init(args=args)
    path_planner_publisher = PathPlanner()
    try:
        rclpy.spin(path_planner_publisher)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        path_planner_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        path_planner_publisher.throttle_and_steering_values.angle = 0.0
        path_planner_publisher.throttle_and_steering_values.throttle = 0.0
        path_planner_publisher.throttle_and_steering_publisher.publish(path_planner_publisher.throttle_and_steering_values)
        time.sleep(1)
        path_planner_publisher.destroy_node()
        rclpy.shutdown()
        path_planner_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
