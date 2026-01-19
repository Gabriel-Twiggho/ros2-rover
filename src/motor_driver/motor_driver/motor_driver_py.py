import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pigpio

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # --- Parameters (default based on setup)
        self.declare_parameter('left_dir_pin', 23)
        self.declare_parameter('left_pwm_pin', 24)
        self.declare_parameter('right_dir_pin', 27)
        self.declare_parameter('right_pwm_pin', 22)

        #kinematics
        self.declare_parameter('wheel_separation', 0.164) # Meters (approx width)
        self.declare_parameter('max_speed', 0.6) # m/s (Software speed cap)
        self.declare_parameter('pwm_frequency', 20000) # Hz (Ultrasonic)       

        # Load parameters
        self.left_dir = self.get_parameter('left_dir_pin').value
        self.left_pwm = self.get_parameter('left_pwm_pin').value
        self.right_dir = self.get_parameter('right_dir_pin').value
        self.right_pwm = self.get_parameter('right_pwm_pin').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.max_speed = self.get_parameter('max_speed').value
        self.freq = self.get_parameter('pwm_frequency').value

        # --- Setup pigpio ---
        self.pi = pigpio.pi()
        #checks if daemon / pigpio is running in background
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpiod daemon!")
            rclpy.shutdown()
            return

        # Initialize Pins, input the settings defined above
        for pin in [self.left_dir, self.left_pwm, self.right_dir, self.right_pwm]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pin, self.freq)
            self.pi.set_PWM_range(pin, 255) # 0-255 duty cycle
            self.pi.write(pin, 0) # Start low
        
        # --- State Variables ---
        self.estop_active = False
        self.last_cmd_time = self.get_clock().now()

        # --- Subscribers ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Latched QoS for estop is safer, but standard is fine for this receiver
        self.create_subscription(Bool, '/estop', self.estop_callback, 10)

        # --- Dead-man Timer (Safety) ---
        # Check every 0.1s. If no command for 0.5s, stop.
        self.create_timer(0.1, self.check_watchdog)

        self.get_logger().info("Motor Driver Initialized. Waiting for commands...")

    def estop_callback(self, msg):
        self.estop_active = msg.data
        if self.estop_active:
            self.get_logger().warn("E-STOP RECEIVED. Halting motors.")
            self.stop_motors()

    def cmd_vel_callback(self, msg):
        # Update watchdog timer
        self.last_cmd_time = self.get_clock().now()

        if self.estop_active:
            return

        # --- Differential Drive Kinematics ---
        # Convert v (linear) and w (angular) to left/right wheel speeds
        linear = msg.linear.x
        angular = msg.angular.z
        
        v_left = linear - (angular * self.wheel_sep / 2.0)
        v_right = linear + (angular * self.wheel_sep / 2.0)

        # Normalize velocities if they exceed max_speed
        max_req = max(abs(v_left), abs(v_right))
        if max_req > self.max_speed:
            scale = self.max_speed / max_req
            v_left *= scale
            v_right *= scale

        self.set_motor(self.left_dir, self.left_pwm, v_left)
        self.set_motor(self.right_dir, self.right_pwm, v_right)

    def set_motor(self, dir_pin, pwm_pin, speed):
        """
        Sets direction and PWM duty cycle for a motor.
        speed: float in m/s (positive = forward, negative = backward)
        """
        # Determine direction (0 or 1) aka backwards/forwards
        # Note: Polarity check happens in G-7. If it's backwards, we flip logic here later.
        direction = 0 if speed >= 0 else 1
        
        # Calculate Duty Cycle (0-255)
        # We map 0..max_speed to 0..255
        duty = int(abs(speed) / self.max_speed * 255)
        duty = min(max(duty, 0), 255) # Clamp

        # Write to hardware
        self.pi.write(dir_pin, direction)
        self.pi.set_PWM_dutycycle(pwm_pin, duty)

    def stop_motors(self):
        self.pi.set_PWM_dutycycle(self.left_pwm, 0)
        self.pi.set_PWM_dutycycle(self.right_pwm, 0)

    def check_watchdog(self):
        # If e-stop is on, we are already stopped.
        if self.estop_active:
            return

        # Calculate time since last command
        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        # Timeout after 0.5 seconds
        if time_diff > 0.5:
            self.stop_motors()


    def on_shutdown(self):
        """Clean up on exit"""
        self.stop_motors()
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
