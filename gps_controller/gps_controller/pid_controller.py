import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pyvesc import VESC
import time
import numpy as np

NODE_NAME = 'pid_controller'
ERROR_TOPIC_NAME = '/error'


class PidController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        # Error subscriber
        self.error_subscriber = self.create_subscription(Float32MultiArray, ERROR_TOPIC_NAME, self.error_measurement, self.QUEUE_SIZE, callback_group=self.error_thread)
        self.error_subscriber

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('integral_max', 0),
                ('error_threshold', 0.15),
                ('zero_speed', 0),
                ('max_speed', 20000),
                ('min_speed', 500),
                ('max_right_steering', 1.0),
                ('max_left_steering', 0.0),
                ('Ts', 0.05)
            ])
        self.Kp = self.get_parameter('Kp_steering').value
        self.Ki = self.get_parameter('Ki_steering').value
        self.Kd = self.get_parameter('Kd_steering').value
        self.integral_max = self.get_parameter('integral_max').value 
        self.error_threshold = self.get_parameter('error_threshold').value # between [0,1]
        self.zero_speed = self.get_parameter('zero_speed').value  # should be around 0
        self.max_speed = self.get_parameter('max_speed').value  # between [0,5] m/s
        self.min_speed = self.get_parameter('min_speed').value  # between [0,5] m/s 
        self.max_right_steering = self.get_parameter('max_right_steering').value  # negative(max_left) 
        self.max_left_steering = self.get_parameter('max_left_steering').value  # between abs([0,0.436332]) radians (0-25degrees)
        self.Ts = self.get_parameter('Ts').value # controller sample time

        # initializing PID control
        self.e_y_buffer = 0
        self.e_y = 0
        self.e_y_1 = 0
        self.custom_data_buffer = 0.0
        self.custom_data = 0.0

        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        
        # Create Vesc Object
        vesc_port = "/dev/ttyACM0"
        try:
            self.vesc = VESC(serial_port=vesc_port, has_sensor=False, start_heartbeat=True, baudrate=115200)
            self.vesc.set_rpm(0)
            self.get_logger().info("Succesfully Connected to VESC!")

        except Exception as e:
            self.get_logger().info(f"Could not connect to VESC, {e}")
            exit()

        self.get_logger().info(
            f'\n Kp_steering: {self.Kp}'
            f'\n Ki_steering: {self.Ki}'
            f'\n Kd_steering: {self.Kd}'
            f'\n Ts: {self.Ts}'
        )
        # Call controller
        self.create_timer(self.Ts, self.controller)

    def error_measurement(self, error_data):
        error_data_check = np.array([error_data.data[0], error_data.data[1], error_data.data[2]])
        if not (np.isnan(error_data_check).any()):
            self.e_y_buffer = error_data.data[2]  # this is delta thetha
            self.e_x_buffer = error_data.data[1]
            self.e_theta_buffer = error_data.data[0]
    
    def get_latest_measurements(self):
        self.e_y = self.e_y_buffer
        self.current_time = self.get_clock().now().to_msg()

    def controller(self):
        # Get latest measurement
        self.get_latest_measurements()

        # Angle PID terms
        self.proportional_error = self.Kp * self.e_y
        self.derivative_error = self.Kd * (self.e_y - self.e_y_1) / self.Ts
        self.integral_error += self.Ki * self.e_y * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)
        delta_normal = self.proportional_error + self.derivative_error + self.integral_error

        # Steering map from [-1,1] --> [0, 1] : [max_left,max_right]
        delta_degree = self.clamp(float(self.remap(delta_normal)), self.max_right_steering, self.max_left_steering)
        self.get_logger().info(f'\n'   
                               f'\n ey:{self.e_y}'
                               f'\n delta:{delta_degree}'
                               )
        self.e_y_1 = self.e_y

        try:
            # publish servo control signal
            self.vesc.set_rpm(int(self.min_speed))
            self.vesc.set_servo(float(delta_degree))

        except KeyboardInterrupt:
            self.vesc.set_rpm(0)
            self.vesc.set_servo(0.5)


    def remap(self, value):
        input_start = -1
        input_end = 1
        output_start = self.max_left_steering
        output_end = self.max_right_steering
        normalized_output = float(output_start + (value - input_start) * ((output_end - output_start) / (input_end - input_start)))
        return normalized_output


    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 


def main(args=None):
    rclpy.init(args=args)
    pid_publisher = PidController()
    try:
        rclpy.spin(pid_publisher)
        pid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        pid_publisher.vesc.set_rpm(0)
        pid_publisher.vesc.set_servo(0.5)
        time.sleep(1)
        pid_publisher.destroy_node()
        rclpy.shutdown()
        pid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

