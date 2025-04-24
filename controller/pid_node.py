import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
import time
from ros2_numpy import pose_to_np, to_ackermann


class PIDcontroller(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # Create a publisher for sending AckermannDriveStamped messages to the '/autonomous/ackermann_cmd' topic
        self.publisher = self.create_publisher(AckermannDriveStamped, '/autonomous/ackermann_cmd', qos_profile)

        # Create a subscription to listen for PoseStamped messages from the '/waypoint' topic
        # When a message is received, the 'self.waypoint_callback' function is called
        self.subscription = self.create_subscription(
            PoseStamped,
            '/waypoint',
            self.waypoint_callback,
            qos_profile
        )

        # Load parameters
        self.params_set = False
        self.declare_params()
        self.load_params()

        # Create a timer that calls self.load_params every 10 seconds (10.0 seconds)
        self.timer = self.create_timer(10.0, self.load_params)

        self.last_error = 0.0
        self.last_time = time.time()

    def waypoint_callback(self, msg: PoseStamped):
        # Convert incoming pose message to position, heading, and timestamp
        point, heading, timestamp_unix = pose_to_np(msg)

        # Calculate time difference since last callback
        # dt = 

        # Update the last time to the current timestamp


        # Get x and y coordinates (ignore z), and compute the error in y
        x, y, _ = point
        error = 0

        # Calculate the derivative of the error (change in error over time)
        # d_error = 

        # Compute the steering angle using a PD controller
        steering_angle = 0.0

        # Get the timestamp from the message header
        timestamp = msg.header.stamp

        # Create an Ackermann drive message with speed and steering angle
        ackermann_msg = to_ackermann(self.speed, steering_angle, timestamp)

        # Publish the message to the vehicle
        self.publisher.publish(ackermann_msg)

        # Save the current error for use in the next iteration
        self.last_error = error


    def declare_params(self):

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.9),
                ('kd', 0.0),
                ('speed', 0.6),
            ]
        )

    def load_params(self):
        try:
            self.kp = self.get_parameter('kp').get_parameter_value().double_value
            self.kd = self.get_parameter('kd').get_parameter_value().double_value
            self.speed = self.get_parameter('speed').get_parameter_value().double_value

            if not self.params_set:
                self.get_logger().info("Parameters loaded successfully")
                self.params_set = True

        except Exception as e:
            self.get_logger().error(f"Failed to load parameters: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = PIDcontroller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
