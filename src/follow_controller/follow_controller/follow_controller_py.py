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
        self.declare_parameter('accel_linear', 0.2)   # m/s^2
        self.declare_parameter('accel_angular', 1.5)  # rad/s^2
    
        self.k_lin = self.get_parameter('k_linear').value
        self.k_ang = self.get_parameter('k_angular').value
        self.accel_lin = self.get_parameter('accel_linear').value
        self.accel_ang = self.get_parameter('accel_angular').value
        self.target_area = self.get_parameter('target_area').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.center_deadband = self.get_parameter('center_deadband').value

        # --- State ---
        self.current_mode = "MANUAL"
        self.estop_active = False
        self.last_bbox_time = 0.0
        self.bbox_timeout = 0.5
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_loop_time = self.time_now()

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

    def reset_state(self):
        """ Instant stop - reset smoothing memeory. """
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
    
    def _apply_slew(self, target, last, max_delta):
        """ 
        Calculating Slew 
        Naming a function _underscore_ is done to show that this function should only be called in this class

        """
        delta = target - last
        delta = max(min(delta, max_delta), -max_delta)
        slewed_target = last + delta
        return slewed_target


    def control_loop(self):
        """
        This control loop does three things, 
        Calculate Target -> distance of person from centre of camera
        Apply Slew -> 
        Publish


        """
        now = self.time_now()
        dt = now - self.last_loop_time
        self.last_loop_time = now

        target_linear = 0.0
        target_angular = 0.0
        
        if self.estop_active or self.current_mode != "FOLLOW":
            self.reset_state()
            self.publish_twist(0.0, 0.0)
            return 

        time_diff = now - self.last_bbox_time
        
        if time_diff <= self.bbox_timeout:
            # P control
            cx = self.current_bbox[0]
            area = self.current_bbox[2]

            error_x = cx - 0.5
            if abs(error_x) > self.center_deadband:
                target_angular = -self.k_ang * error_x

            error_area = self.target_area - area
            target_linear = self.k_lin * error_area

            # Hard clamp targets
            target_linear = max(min(target_linear, self.max_lin), -self.max_lin)
            target_angular = max(min(target_angular, self.max_ang), -self.max_ang)
            
        # Calculate max allowed change this loop
        max_delta_lin = self.accel_lin * dt
        max_delta_ang = self.accel_ang * dt

        #Apply Slew
        smooth_linear = self._apply_slew(target_linear,self.last_linear_x, max_delta_lin)
        smooth_angular = self._apply_slew(target_angular, self.last_angular_z, max_delta_ang)

        self.publish_twist(smooth_linear, smooth_angular)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

        # save for next loop
        self.last_linear_x = linear
        self.last_angular_z = angular

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


