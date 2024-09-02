import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from px4_msgs.msg import VehicleLocalPosition
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np
import math

class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')


        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('update_rate', 0.1)

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # Initialize state vector [x, y, v_x, v_y, yaw]
        self.state = np.zeros(5)
        self.P = np.eye(5)  # Covariance matrix

        # EKF process noise and measurement noise matrices
        self.Q = np.eye(5) * 0.01  # Process noise
        self.R = np.eye(3) * 0.1   # Measurement noise

        # Subscribers for IMU and Wheel Odometry data
        #self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/wheel/odometry', self.odom_callback, 10)

        self.imu_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.imu_callback, qos_profile)
        
        # Publisher for EKF output
        self.ekf_pub = self.create_publisher(Odometry, '/ekf/odom', 10)

        # Timer to run the EKF update step
        self.timer = self.create_timer(self.update_rate, self.run_ekf)

        # Initialize time and other state variables
        self.last_time = self.get_clock().now()
        self.last_yaw = 0.0

        self.altitude = 0.0

         # Static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish a static transform between 'odom' and 'base_link'
        self.publish_static_transform()

       
    def imu_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        yaw = - vehicle_local_position.heading
        self.altitude = - vehicle_local_position.z
        self.state[4] = yaw

    def odom_callback(self, msg):
        # Extract velocity from wheel odometry
        v_x_local = msg.twist.twist.linear.x
        v_y_local = msg.twist.twist.linear.y

        # Get the yaw from the state (updated in imu_callback)
        yaw = self.state[4]

        # Transform local velocities to global frame using the yaw
        v_x_global = v_x_local * np.cos(yaw) - v_y_local * np.sin(yaw)
        v_y_global = v_x_local * np.sin(yaw) + v_y_local * np.cos(yaw)

        # Update state velocities in global frame
        self.state[2] = v_x_global
        self.state[3] = v_y_global

    def euler_from_quaternion(self, quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def run_ekf(self):
        # Time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Predict step
        self.predict(dt)

        # Update step (using IMU and odometry data)
        self.update()

        # Publish EKF output
        self.publish_ekf()

    def predict(self, dt):
        # State transition model
        F = np.eye(5)
        F[0, 2] = dt
        F[1, 3] = dt

        # Predict the next state
        self.state = F @ self.state

        # Predict the next covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self):
        # Measurement matrix
        H = np.zeros((3, 5))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 4] = 1

        # Measurement residual
        Z = np.array([self.state[0], self.state[1], self.state[4]])
        Y = Z - H @ self.state

        # Residual covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state estimate
        self.state += K @ Y

        # Update covariance matrix
        self.P = (np.eye(5) - K @ H) @ self.P

    def publish_ekf(self):
        # Publish the EKF estimated odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = self.altitude 

        # Convert yaw to quaternion
        quat = self.quaternion_from_euler(0, 0, self.state[4])
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Publish the odometry message
        self.ekf_pub.publish(odom)
    
    def publish_static_transform(self):
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'map'
        static_transform_stamped.child_frame_id = 'base_link'

        # Example: Identity transform (no translation or rotation)
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0

        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(static_transform_stamped)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
