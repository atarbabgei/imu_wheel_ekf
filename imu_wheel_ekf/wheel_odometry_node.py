import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
import math

class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # Parameters (tune these to your robot's specifications)
        self.wheel_radius = 0.125  # wheel radius in meters

        # Initialize state variables
        self.x = 0.0  # Robot's x position
        self.y = 0.0  # Robot's y position
        self.theta = 0.0  # Robot's orientation (yaw)
        
        # Initialize velocity variables
        self.angular_velocity = 0.0  # Angular velocity (initialize to 0.0)
        self.cumulative_angle = 0.0  # Cumulative angle (initialize to 0.0)

        # Subscribe to encoder topics
        self.angle_sub = self.create_subscription(Float32, '/encoder/absolute_angle', self.angle_callback, 10)
        self.velocity_sub = self.create_subscription(Float32, '/encoder/angular_velocity', self.velocity_callback, 10)
        self.cumulative_angle_sub = self.create_subscription(Float32, '/encoder/cumulative_angle', self.cumulative_angle_callback, 10)

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odometry', 10)
        
        # Timer to publish odometry data
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        
        # State variables for calculations
        self.last_time = self.get_clock().now()
        self.last_angle = 0.0

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def angle_callback(self, msg):
        # Update the absolute angle (not used directly in this example)
        self.last_angle = msg.data

    def velocity_callback(self, msg):
        # Update the angular velocity (used to calculate linear velocity)
        self.angular_velocity = msg.data

    def cumulative_angle_callback(self, msg):
        # Update cumulative angle (could be used for position, but here just for completeness)
        self.cumulative_angle = msg.data

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

        # Example calculation for a wheel-driven robot
        linear_velocity = self.angular_velocity * self.wheel_radius
        
        # Update position (x, y) based on the velocities
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        # Create and populate the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation
        quat = self.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Set velocities
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        # Publish odometry
        self.odom_pub.publish(odom)
        
         # Update last time
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
