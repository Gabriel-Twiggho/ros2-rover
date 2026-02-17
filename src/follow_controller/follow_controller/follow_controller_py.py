import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray

class FollowControllerNode(Node):
    def __init__(self):
        super().__init__('follow_controller_node')

        # --- Parameters ---
        self.declare_parameter('k_linear', 1.2)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('target_area', 0.7)
        self.declare_parameter('safe_area', 0.30)
        self.declare_parameter('center_deadband', 0.05)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 1.0)
    
        self.k_lin = self.get_parameter('k_linear').value
        self.k_ang = self.get_parameter('k_angular').value
        self.target_area = self.get_parameter('target_area').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value

        # --- State ---
        self.current_mode = "MANUAL"
        self.estop_active = False
        self.last_bbox_time = 0.0
        self.bbox_timeout = 0.5

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- Publishers / Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.create_subscription(String, '/sys/mode', self.mode_callback, latched_qos)
        self.create_subscription(Bool, '/estop', self.estop_callback, latched_qos)
        self.create_subscription(Float32MultiArray, '/person_bbox', self.bbox_callback, 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Follow Controller Initialized.")

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def estop_callback(self, msg):
        self.estop_active = msg.data 

    def bbox_callback(self, msg):
        if len(msg.data) < 4:
            return
        self.current_bbox = msg.data
        self.last_bbox_time = self.time_now()
    
    def time_now(self):
        return self.get_clock().now().nanoseconds / 1e9
    
    def control_loop(self):
        twist = Twist()

        if self.current_mode != "FOLLOW":
            self.cmd_pub.publish(twist)
            return

        time_diff = self.time_now() - self.last_bbox_time
        if time_diff > self.bbox_timeout:
            self.cmd_pub.publish(twist)
            return

        cx = self.current_bbox[0]
        area = self.current_bbox[2]
        error_x = cx - 0.5
        deadband = self.get_parameter('center_deadband').value
        if abs(error_x) > deadband:
            twist.angular.z = -self.k_ang * error_x

        error_area = self.target_area - area
        twist.linear.x = self.k_lin * error_area
        
        # current_bbox: [0]=cx (horizontal center 0-1), [1]=cy (vertical), [2]=area (size 0-1), [3]=conf
        twist.linear.x = max(min(twist.linear.x, self.max_lin), -self.max_lin)
        twist.angular.z = max(min(twist.angular.z, self.max_ang), -self.max_ang)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowControllerNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()


