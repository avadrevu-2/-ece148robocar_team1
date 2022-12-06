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
        self.error_subscriber = self.create_subscription(Float32MultiArray, ERROR_TOPIC_NAME, self.error_measurement, self.QUEUE_SIZE)
        self.error_subscriber

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0.01),
                ('Kp_heading', 0.4),
                ('Kd_heading', 0.001),
                ('integral_max', 0),
                ('error_threshold', 0.15),
                ('zero_speed', 0),
                ('max_speed', 20000),
                ('min_speed', 500),
                ('max_right_steering', 1.0),
                ('max_left_steering', 0.0),
                ('no_steering', 0.5),
                ('refresh_hz', 20.0)
            ])
        self.Kp = self.get_parameter('Kp_steering').value
        self.Ki = self.get_parameter('Ki_steering').value
        self.Kd = self.get_parameter('Kd_steering').value
        self.Kp_heading = self.get_parameter('Kp_heading').value
        self.Kd_heading = self.get_parameter('Kd_heading').value
        self.integral_max = self.get_parameter('integral_max').value 
        self.error_threshold = self.get_parameter('error_threshold').value # between [0,1]
        self.zero_speed = self.get_parameter('zero_speed').value  # should be around 0
        self.max_speed = self.get_parameter('max_speed').value  # between [0,5] m/s
        self.min_speed = self.get_parameter('min_speed').value  # between [0,5] m/s 
        self.max_right_steering = self.get_parameter('max_right_steering').value 
        self.max_left_steering = self.get_parameter('max_left_steering').value 
        self.no_steering = self.get_parameter('no_steering').value
        self.Ts = 1.0 / self.get_parameter('refresh_hz').value # controller sample time

        # initializing PID control
        self.e_y_buffer = 0
        self.e_y = 0
        self.e_y_1 = 0
        self.e_theta = 0
        self.e_theta_1 = 0
        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        self.heading_proportional = 0  # proportional term for heading
        self.heading_derivative = 0  # proportional term for 
        
        # Create Vesc Object
        vesc_port = "/dev/ttyACM0"
        try:
            self.vesc = VESC(serial_port=vesc_port, has_sensor=False, start_heartbeat=True, baudrate=115200)
            self.vesc.set_rpm(0)
            self.get_logger().info("Succesfully Connected to VESC!")

        except Exception as e:
            self.get_logger().info(f"Could not connect to VESC, {e}")
            rclpy.shutdown()

        self.get_logger().info(
            f'\n Kp_steering: {self.Kp}'
            f'\n Ki_steering: {self.Ki}'
            f'\n Kd_steering: {self.Kd}'
            f'\n Kp_heading: {self.Kp_heading}'
            f'\n Kd_heading: {self.Kd_heading}'
            f'\n Ts: {self.Ts}'
        )
        # Call controller
        self.create_timer(self.Ts, self.controller)


    def error_measurement(self, error_data):
        """
        Get data from error_data message
        """
        error_data_check = np.array([error_data.data[0], error_data.data[1]])
        if not (np.isnan(error_data_check).any()):
            self.e_y_buffer = error_data.data[0]  # this is cross_track error
            self.e_theta_buffer = error_data.data[1]  # this is heading error

    
    def get_latest_measurements(self):
        """
        Adds data from buffer to variables
        """
        self.e_y = self.e_y_buffer
        self.e_theta = self.e_theta_buffer
        self.current_time = self.get_clock().now().to_msg()


    def controller(self):
        """
        Main PID controller, will loop until error values
        are set really high. Does PID calculations and controls
        motor.
        """
        # Get latest measurement
        self.get_latest_measurements()

        # If the error values are 100000.0, that means stop controlling
        if self.e_y == 100000.0 or self.e_theta == 100000.0:
            self.get_logger.info("Not Controlling, Error too high")
            self.vesc.set_rpm(int(self.zero_speed))
            self.vesc.set_servo(self.no_steering)
            return
        
        # Cross Track PID terms
        self.proportional_error = self.Kp * self.e_y
        self.derivative_error = self.Kd * (self.e_y - self.e_y_1) / self.Ts
        self.integral_error += self.Ki * self.e_y * self.Ts
        self.integral_error = self.clamp(self.integral_error, self.integral_max)

        # Heading PID terms
        self.heading_proportional = self.Kp_heading * self.e_theta
        self.heading_derivative = self.Kd_heading * (self.e_theta - self.e_theta_1) / self.Ts

        # Calculate Delta and map it from 0 to 1
        delta_normal = self.proportional_error + self.derivative_error + self.integral_error + self.heading_proportional + self.heading_derivative
        delta_degree = self.clamp(float(self.remap(delta_normal)), self.max_right_steering, self.max_left_steering)
        self.get_logger().info(f'\n'   
                               f'\n e_cross_track: {self.e_y}'
                               f'\n e_theta: {self.e_theta}'
                               f'\n steering command: {delta_degree}'
                               )
        
        # Store previous errors for derivative 
        self.e_y_1 = self.e_y
        self.e_theta_1 = self.e_theta

        try:
            # Command the vesc
            self.vesc.set_rpm(int(self.min_speed))
            self.vesc.set_servo(float(delta_degree))

        except KeyboardInterrupt:
            self.vesc.set_rpm(int(self.zero_speed))
            self.vesc.set_servo(self.no_steering)


    def remap(self, value):
        """
        Remap value from [-1 to 1] into [0 to 1]
        """
        input_start = -1
        input_end = 1
        output_start = self.max_left_steering
        output_end = self.max_right_steering
        normalized_output = float(output_start + (value - input_start) * ((output_end - output_start) / (input_end - input_start)))
        return normalized_output


    def clamp(self, value, upper_bound, lower_bound=None):
        """
        Clamps input value to upper and lower bound
        """
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 


    def shutdown_vesc(self):
        """
        Shuts down communication with VESC
        """
        self.vesc.__exit__(None, None, None)
        self.get_logger().info("Shutdown VESC")


def main(args=None):
    rclpy.init(args=args)
    pid_publisher = PidController()
    try:
        rclpy.spin(pid_publisher)
        pid_publisher.shutdown_vesc()
        pid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        pid_publisher.vesc.set_rpm(int(pid_publisher.zero_speed))
        pid_publisher.vesc.set_servo(pid_publisher.no_steering)
        time.sleep(1)
        pid_publisher.shutdown_vesc()
        time.sleep(1)
        pid_publisher.destroy_node()
        rclpy.shutdown()
        pid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

