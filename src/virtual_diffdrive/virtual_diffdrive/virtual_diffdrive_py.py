# --- Import required libraries ---
import rclpy  # ROS2 client library
from rclpy.node import Node  # Base class for ROS2 nodes
from rclpy.qos import QoSProfile, DurabilityPolicy # Import QoS for the subscriber
from geometry_msgs.msg import Twist, TransformStamped  # Velocity + TF messages
from nav_msgs.msg import Odometry  # Odometry message type
from std_msgs.msg import Bool # Import Bool for the /estop message
from tf2_ros import TransformBroadcaster  # For broadcasting TF transforms
import math  # Math functions
import time  # Time functions (not heavily used here)


def euler_to_quaternion(yaw):
    """Converts a yaw angle to a quaternion (since TF needs quaternion orientation)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = 1  # cos(pitch * 0.5), for 2D robot pitch = 0
    sp = 0  # sin(pitch * 0.5)
    cr = 1  # cos(roll * 0.5), for 2D robot roll = 0
    sr = 0  # sin(roll * 0.5)

    q = [0.0] * 4
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    return q


class VirtualDiffDriveNode(Node):
    """Simulates a differential drive robot by integrating /cmd_vel commands."""

    def __init__(self):
        super().__init__('virtual_diffdrive_node')

        # --- Parameters (tunable update rates) ---
        self.declare_parameter('integration_rate', 50.0)   # Hz, how often pose is updated
        self.declare_parameter('odom_publish_rate', 20.0)  # Hz, how often odom is published

        integration_hz = self.get_parameter('integration_rate').value
        odom_hz = self.get_parameter('odom_publish_rate').value

        # --- State Variables (robot's pose and velocity) ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()  # Timestamp for integration step
        self.estop_active = False # NEW: E-Stop state variable

        #Basically creating a queue profile for the estop_subscriber, this will alter the queue to 1 but it will keep the last incoming msg
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- ROS2 Components ---
        # Subscribe to /cmd_vel to receive velocity commands
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        #subscribe to the /estop topic (which publishes bool messages, estop_callback is triggered when message is received)
        self.estop_subscriber = self.create_subscription(
            Bool, '/estop', self.estop_callback, latched_qos) # Use latched QoS

        # Publisher for /odom topic
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster (odom -> base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers: one for integration, one for publishing
        self.create_timer(1.0 / integration_hz, self.update_pose)
        self.create_timer(1.0 / odom_hz, self.publish_all)

        self.get_logger().info("Virtual Differential Drive is ready.")

    def estop_callback(self, msg):
        """NEW: Callback for /estop topic."""
        if msg.data and not self.estop_active:
            self.get_logger().warn('E-Stop ACTIVATED. Halting motion.')
            # Immediately zero out velocities
            self.vx = 0.0
            self.vth = 0.0
        elif not msg.data and self.estop_active:
            self.get_logger().info('E-Stop DEACTIVATED. Resuming normal operation.')
        
        self.estop_active = msg.data

    def cmd_vel_callback(self, msg):
        """Store the latest velocity commands, only if E-Stop is not active."""
        if self.estop_active:
            # If e-stopped, ensure velocities are zero and ignore the command.
            self.vx = 0.0
            self.vth = 0.0
            return 
        
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_pose(self):
        """Updates the robot's pose using Euler integration."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert ns -> seconds
        self.last_time = current_time

        # Integrate velocities to compute position change
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th
        # --- TEMPORARY DEBUG LINE ---
        #self.get_logger().info(f'Integration loop dt: {dt * 1000:.2f} ms')
    def publish_all(self):
        """Publishes odometry and TF transform."""
        current_time = self.get_clock().now()
        
        # --- TF Transform (odom -> base_link) ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = euler_to_quaternion(self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # --- Odometry Message ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose (position + orientation)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity (linear + angular)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        # Publish odometry
        self.odom_publisher.publish(odom)


def main(args=None):
    """ROS2 main entry point."""
    rclpy.init(args=args)
    node = VirtualDiffDriveNode()
    try:
        rclpy.spin(node)  # Keep node alive
    except KeyboardInterrupt:
        pass  # Graceful exit on Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
