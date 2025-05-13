import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
import time
from ros2_numpy import pose_to_np, to_ackermann
from collections import deque
import numpy as np
import time
from controller.targets import Target

class Vehicle():
    def __init__(self, id, label, min_distance = 1.0, max_distance = 2.5):
        self.id = id          # class ID
        self.label = label    # Descriptive label for the target (e.g., 'stop_sign')
        self.min_distance = min_distance
        self.max_distance = max_distance

    def update(self, position, node):
        x = position[0]
        if x < self.min_distance:
            node.speed = 0
            node.get_logger().info(f"Leading vehicle too close at {x:.2f} meters. Stopping.")
        else:
            # Compute how far we are relative to the minimum safe distance (clamped between 0 and 1)
            distance_ratio = max(0.0, min((x - self.min_distance) / (self.max_distance - self.min_distance), 1.0))

            # Interpolate speed between min_speed and max_speed
            node.speed = node.min_speed + distance_ratio * (node.max_speed - node.min_speed)

            node.get_logger().info(f"Leading vehicle at {x:.2f} meters. Adjusting speed to {node.speed:.2f}.")

class StopSign(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5, duration=2.0):
        super().__init__(id, label, threshold, len_history, min_distance)
        self.duration = duration

    def react(self, node):
        node.get_logger().info(f"Stopping for stop sign at x={self.position[0]:.2f}")
        msg = to_ackermann(0.0, node.last_steering_angle)
        node.publisher.publish(msg)
        time.sleep(self.duration)
        node.get_logger().info(f"Resume driving...")
        return True

class SpeedSign(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5, speed=0.6):
        super().__init__(id, label, threshold, len_history, min_distance)
        self.speed = speed

    def react(self, node):
        node.get_logger().info(f"Setting speed to {self.speed:.2f} m/s at x={self.position[0]:.2f}")
        node.speed = self.speed  # Assuming this is consumed somewhere else in your control loop
        return True

class RedLight(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5):
        super().__init__(id, label, threshold, len_history, min_distance)

    def react(self, node):
        node.get_logger().info(f"Red light detected at x={self.position[0]:.2f}. Stopping...")
        node.speed = 0.0  # Stop vehicle
        return True

class GreenLight(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5):
        super().__init__(id, label, threshold, len_history, min_distance)

    def react(self, node):
        # Green light in range — resume driving
        node.get_logger().info(f"Green light detected at x={self.position[0]:.2f}. Resume driving...")
        node.speed = node.max_speed  # Resume driving
        return True

class PIDcontroller(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # Create a publisher for sending AckermannDriveStamped messages to the '/autonomous/ackermann_cmd' topic
        self.publisher = self.create_publisher(AckermannDriveStamped, 'autonomous/ackermann_cmd', qos_profile)

        # Create a subscription to listen for PoseStamped messages from the '/waypoint' topic
        # When a message is received, the 'self.waypoint_callback' function is called

        self.way_sub = self.create_subscription(PoseStamped,'/waypoint', self.waypoint_callback, qos_profile)
        self.obj_sub = self.create_subscription(PoseStamped,'/objects', self.obj_callback, qos_profile)

        # Load parameters
        self.params_set = False
        self.declare_params()
        self.load_params()

        # Create a timer that calls self.load_params every 10 seconds (10.0 seconds)
        self.timer = self.create_timer(10.0, self.load_params)

        self.last_error = 0.0
        self.last_time = time.time()
        self.last_steering_angle = 0.0

        # Initialize deque with a fixed length of self.max_out
        # This could be useful to allow the vehicle to temporarily lose the track for up to max_out frames before deciding to stop. (Currently not used yet.)
        self.max_out = 9
        self.success = deque([True] * self.max_out, maxlen=self.max_out)


        self.id2target = {
            0: Vehicle(0, 'car'),
            1: StopSign(1, 'stop'),
            2: SpeedSign(2, 'speed_2mph', speed = 0.6),
            3: SpeedSign(3, 'speed_3mph', speed = 0.8),
            4: GreenLight(4, 'green'),
            5: RedLight(4, 'red'),
        }

  

    def obj_callback(self, msg: PoseStamped):

        # Convert incoming pose message to position, heading, and timestamp
        point, heading, timestamp_unix = pose_to_np(msg)

        id = int(point[-1]) # Extract class ID stored in the z-coordinate

        self.id2target[id].update(point, self)

    def waypoint_callback(self, msg: PoseStamped):
        
        # Convert incoming pose message to position, heading, and timestamp
        point, heading, timestamp_unix = pose_to_np(msg)

        # If the detected point contains NaN (tracking lost) stop the vehicle
        if np.isnan(point).any():
            self.success.append(False)
            if any(self.success):
                # Keep driving with the last known steering angle unless the line is lost for self.max_out consecutive frames
                ackermann_msg = to_ackermann(self.speed, self.last_steering_angle, timestamp_unix)
            else:
                ackermann_msg = to_ackermann(0.0, self.last_steering_angle, timestamp_unix)
                self.publisher.publish(ackermann_msg) # BRAKE
            return
        else:
            self.success.append(True)


        # Calculate time difference since last callback
        dt = timestamp_unix - self.last_time

        # Update the last time to the current timestamp
        self.last_time = timestamp_unix

        # Get x and y coordinates (ignore z), and compute the error in y
        x, y, _ = point
        error = 0.0 - y  # Assuming the goal is to stay centered at y = 0

        # Calculate the derivative of the error (change in error over time)
        d_error = (error - self.last_error) / dt 

        # Compute the steering angle using a PD controller
        steering_angle = (self.kp * error) + (self.kd * d_error)

        # Get the timestamp from the message header
        timestamp = msg.header.stamp

        # Create an Ackermann drive message with speed and steering angle
        ackermann_msg = to_ackermann(self.speed, steering_angle, timestamp)

        # Publish the message to the vehicle
        self.publisher.publish(ackermann_msg)

        # Save the current error for use in the next iteration
        self.last_error = error

        # If tracking is lost, continue driving with the last known steering angle
        # to follow the previously estimated path (e.g., maintain the current curve)
        self.last_steering_angle = steering_angle


    def declare_params(self):

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.9),
                ('kd', 0.0),
                ('max_speed', 0.8),
                ('min_speed', 0.4),
            ]
        )

    def load_params(self):
        try:
            self.kp = self.get_parameter('kp').get_parameter_value().double_value
            self.kd = self.get_parameter('kd').get_parameter_value().double_value
            self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
            self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
            self.speed = self.max_speed

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
