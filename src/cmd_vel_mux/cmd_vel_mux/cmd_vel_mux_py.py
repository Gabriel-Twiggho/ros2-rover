import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray

class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')


        self.current_mode = "MANUAL"
        self.estop_active = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/sys/mode', self.mode_callback, latched_qos)
        self.create_subscription(Bool, '/estop', self.estop_callback, latched_qos)
        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.auto_callback, 10)

    def mode_callback(self, msg):
        previous_mode = self.current_mode
        self.current_mode = msg.data
        if previous_mode != self.current_mode:
            self.get_logger().info(f"Mode switched: {previous_mode} -> {self.current_mode}")
            self.publish_zero()
    
    def estop_callback(self, msg):
        self.estop_active = msg.data
        if self.estop_active:
            self.publish_zero()

    def manual_callback(self, msg):
        if self.estop_active:
            return
        if self.current_mode == "MANUAL":
            self.cmd_pub.publish(msg)
        
    def auto_callback(self, msg):
        if self.estop_active:
            return
        if self.current_mode == "FOLLOW":
            self.cmd_pub.publish(msg)
        
    def publish_zero(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
