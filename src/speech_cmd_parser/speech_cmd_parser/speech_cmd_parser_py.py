# --- Import required libraries ---
import rclpy  # ROS2 client library for Python
from rclpy.node import Node  # Base class for creating ROS2 nodes
from rclpy.qos import QoSProfile, DurabilityPolicy  # For configuring Quality of Service
from std_msgs.msg import String, Bool  # Standard ROS2 message types (speech commands + estop state)
from geometry_msgs.msg import Twist  # Message type for velocity commands
from word2number import w2n #converts numbers to digits
import re  # Regex module (used to parse numbers from speech commands)
import subprocess #allows us to run raspi console commands from this script in python



# --- Define the custom ROS2 node ---
class SpeechCommandParserNode(Node):
    def __init__(self):
        # Initialize the node with the name "speech_cmd_parser_node"
        super().__init__('speech_cmd_parser_node')

        # --- Parameters (configurable at runtime) ---
        self.declare_parameter('linear_speed', 0.25)   # Robot's forward/backward speed in m/s
        self.declare_parameter('angular_speed', 2.4)   # Robot's turning speed in rad/s
        self.declare_parameter('default_duration', 1.0)  # Default motion duration in seconds
        self.declare_parameter('max_duration', 10)      # Maximum allowed duration in seconds

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.default_duration = self.get_parameter('default_duration').value
        self.max_duration = self.get_parameter('max_duration').value

        #global variable ----------------
        self.vel = Twist()

        # --- State Variables ---
        self.estop_latched = False       # Tracks whether emergency stop is active
        self.motion_timer = None         # Holds the motion timer (dead-man's switch)

        # creates QOS profile, Publisher sends message → Subscriber receives it, Publisher keeps last message in memory, If subscriber connects later → Gets the LAST message immediately! With Queue size of 1
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- Publishers ---
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_publisher = self.create_publisher(Bool, '/estop', latched_qos)

        # --- Subscribers ---
        self.speech_subscription = self.create_subscription(
            String, '/speech_text', self.speech_callback, 10
        )

        #create timer that calls to publish velocity and also checks until when to stop velocity
        self.create_timer(0.1, self.pub_vel)

        self.get_logger().info("Speech Command Parser is ready.")
        self.publish_estop_state()

    # --- Callback for handling incoming speech commands ---
    def speech_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'Received original command: "{command}"')
        
        # We only log the converted command if it actually changed.
        if command != msg.data.lower():
            self.get_logger().info(f'Converted command to: "{command}"')

        # --- E-Stop Logic ---
        if 'stop' in command:
            self.get_logger().warn('EMERGENCY STOP command received!')
            self.estop_latched = True
            self.stop_motion()
            self.publish_estop_state()
            return

        if 'resume' in command:
            self.get_logger().info('Resume command received.')
            self.estop_latched = False
            self.publish_estop_state()
            return

        if 'shut' in command and 'down' in command and 'confirm' in command:
            self.get_logger().info('Command to shutdown robot in process...')
            self.stop_motion()
            subprocess.run(["sudo", "shutdown", "now"])
            return 

        if self.estop_latched:
            self.get_logger().info('E-Stop is active. Ignoring command. Say "resume" to clear.')
            return
        

        # --- Motion Command Parsing ---
        twist_msg = Twist()
        duration = self.default_duration

        # --- Extract numbers from command if they exist ---
        command_number = self.get_number(command)
        if command_number is not None:
            if 'left' in command or 'right' in command:
                duration = (command_number * 3.14159 / 180.0) / self.angular_speed
            else:
                duration = command_number

        duration = min(duration, self.max_duration)

        # --- Map speech keywords to robot motion ---
        if 'forward' in command:
            twist_msg.linear.x = self.linear_speed
        elif 'backward' in command or 'back' in command:
            twist_msg.linear.x = -self.linear_speed
        elif 'left' in command:
            twist_msg.angular.z = self.angular_speed
        elif 'right' in command:
            twist_msg.angular.z = -self.angular_speed
        elif 'stop' in command:
            self.stop_motion()
            return
        else:
            return

        self.vel = twist_msg
        self.get_logger().info(f"Executing motion for {duration:.2f} seconds.")

        # --- Dead-man’s switch timer ---
        if self.motion_timer is not None:
            self.motion_timer.cancel()
        self.motion_timer = self.create_timer(duration, self.stop_motion)

    def pub_vel(self):
        twist_msg = self.vel
        self.cmd_vel_publisher.publish(twist_msg)


    # --- Helper method to stop the robot ---
    def stop_motion(self):
        if self.motion_timer is not None:
            self.motion_timer.cancel()
        twist_msg = Twist()
        self.cmd_vel_publisher.publish(twist_msg)
        self.vel = Twist()
        self.get_logger().info("Motion stopped (timer expired or stop command).")

    # --- Helper method to publish current e-stop state ---
    def publish_estop_state(self):
        estop_msg = Bool()
        estop_msg.data = self.estop_latched
        self.estop_publisher.publish(estop_msg)
        
    def get_number(self, command):
        # Try to extract number words from the command
        words = command.split()
        for i in range(len(words)):
            try:
                # Start with the longest possible phrase (up to 4 words) and reduce size
                for j in range(min(i+4, len(words)+1), i+1, -1):  # Reverse range
                    phrase = ' '.join(words[i:j])
                    num = w2n.word_to_num(phrase)
                    print(num)
                    return float(num)
            except ValueError:
                continue
        # Fallback: extract digits if present
        match = re.search(r'(\d+\.?\d*)', command)
        if match:
            return float(match.group(1))
        return None  

def main(args=None):
    rclpy.init(args=args)
    node = SpeechCommandParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
        node.get_logger().info('Performing final command')
        node.stop_motion()

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
