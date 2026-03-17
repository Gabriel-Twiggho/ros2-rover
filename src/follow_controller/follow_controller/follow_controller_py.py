import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray


class FollowControllerNode(Node):
    """
    Visual person-following controller for a differential-drive rover.

    Workflow
    --------
    1. A separate YOLO node on the PC detects a person in the Pi camera feed
       and publishes a normalised bounding box to /person_bbox.
    2. bbox_callback receives [cx, cy, area, confidence], rejects low-confidence
       detections, and applies EMA smoothing to cx and area.
    3. control_loop runs at 10 Hz and computes:
       - Angular PD-control: steers the rover so the person stays centred.
       - Linear P-control:   drives forward/backward to keep the person at the
         desired size (target_area) in frame.
    4. Outputs are deadbanded (ignore tiny errors), clamped to min/max velocity,
       and slew-rate limited before publishing to /cmd_vel_auto.

    Tuning Guide (PD Angular)
    -------------------------
    Start with kd_angular = 0 (pure P) and a moderate k_angular (~2-4).
      - If the rover oscillates side-to-side around the target: increase kd_angular
        in steps of 0.2 until the oscillation damps out.
      - If the rover feels sluggish and slow to start turning: kd_angular is too
        high — back it off.
      - If the rover doesn't turn hard enough to keep up: raise k_angular.
      - If the rover overshoots and hunts: lower k_angular OR raise kd_angular.

    Tuning Guide (P Linear)
    -----------------------
    k_linear controls how aggressively the rover chases. target_area is the
    bounding-box area fraction where the rover is "happy" (stops driving).
      - Rover creeps too slowly toward the person: raise k_linear.
      - Rover overshoots and gets too close: lower k_linear or raise safe_area.
      - min_linear should be just above the stall speed of your drive motors.

    General Order
    -------------
    1. Tune angular first (person stationary, rover rotating only).
    2. Then tune linear (person walking straight away/toward the rover).
    3. Finally test combined motion and adjust min/max limits.
    """

    def __init__(self):
        super().__init__('follow_controller_node')

        # --- Control gains ---
        self.declare_parameter('k_linear', 1.2)
        self.declare_parameter('k_angular', 3.0)
        self.declare_parameter('kd_angular', 0.5)

        # --- Target / safety thresholds (normalised bbox area, 0-1) ---
        self.declare_parameter('target_area', 0.7)
        self.declare_parameter('safe_area', 1.0)

        # --- Velocity limits ---
        self.declare_parameter('center_deadband', 0.05)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 2.5)
        self.declare_parameter('min_linear', 0.08)
        self.declare_parameter('min_angular', 0.7)
        self.declare_parameter('accel_linear', 0.6)    # m/s^2
        self.declare_parameter('accel_angular', 4.0)   # rad/s^2

        # --- Perception filtering ---
        self.declare_parameter('min_confidence', 0.30)
        self.declare_parameter('ema_alpha', 0.4)       # 0 = ignore new, 1 = no smoothing

        self.k_lin = self.get_parameter('k_linear').value
        self.k_ang = self.get_parameter('k_angular').value
        self.kd_ang = self.get_parameter('kd_angular').value
        self.target_area = self.get_parameter('target_area').value
        self.safe_area = self.get_parameter('safe_area').value
        self.center_deadband = self.get_parameter('center_deadband').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.min_lin = self.get_parameter('min_linear').value
        self.min_ang = self.get_parameter('min_angular').value
        self.accel_lin = self.get_parameter('accel_linear').value
        self.accel_ang = self.get_parameter('accel_angular').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.ema_alpha = self.get_parameter('ema_alpha').value

        # --- State ---
        self.current_mode = "MANUAL"
        self.estop_active = False
        self.smooth_cx = 0.5
        self.smooth_area = 0.0
        self.prev_error_x = 0.0
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

    # ── Subscription callbacks ──────────────────────────────────────────

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def estop_callback(self, msg):
        self.estop_active = msg.data

    def bbox_callback(self, msg):
        """
        Receive a person detection and update smoothed perception state.

        Expected format: [cx, cy, area, confidence]
        All values normalised 0-1 relative to frame dimensions.

        # TODO: Once the YOLO node on the PC is updated to use model.track()
        #   instead of model.predict(), the message format will become:
        #   [cx, cy, area, confidence, track_id]
        #
        #   Changes needed here:
        #   1. Check len(msg.data) < 5 instead of < 4.
        #   2. Add self.locked_track_id = None to __init__ state and reset_state().
        #   3. On first valid detection, set self.locked_track_id = int(msg.data[4]).
        #   4. On subsequent detections, reject if track_id != self.locked_track_id.
        #   5. When bbox times out (person lost), reset locked_track_id to None
        #      so the rover re-acquires the next person it sees.
        """
        if len(msg.data) < 4:
            return

        confidence = msg.data[3]
        if confidence < self.min_confidence:
            self.get_logger().debug(
                f'Ignoring low-confidence detection: {confidence:.2f}')
            return

        self.last_bbox_time = self.time_now()

        # EMA smoothing — reduces frame-to-frame jitter from YOLO
        alpha = self.ema_alpha
        self.smooth_cx = alpha * msg.data[0] + (1.0 - alpha) * self.smooth_cx
        self.smooth_area = alpha * msg.data[2] + (1.0 - alpha) * self.smooth_area

    # ── Helpers ─────────────────────────────────────────────────────────

    def time_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def reset_state(self):
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.smooth_cx = 0.5
        self.smooth_area = 0.0
        self.prev_error_x = 0.0

    def _apply_slew(self, target, previous, max_delta):
        delta = max(min(target - previous, max_delta), -max_delta)
        return previous + delta

    def _apply_deadband_with_min(self, value, deadband, minimum):
        """Return 0 inside the deadband, else clamp magnitude >= minimum."""
        if abs(value) <= deadband:
            return 0.0
        if abs(value) < minimum:
            return minimum if value > 0 else -minimum
        return value

    # ── Main control loop (10 Hz) ──────────────────────────────────────

    def control_loop(self):
        now = self.time_now()
        dt = now - self.last_loop_time
        self.last_loop_time = now

        if dt <= 0.0:
            return

        target_linear = 0.0
        target_angular = 0.0

        # Gate: only run when in FOLLOW mode and estop is clear
        if self.estop_active or self.current_mode != "FOLLOW":
            self.reset_state()
            self.publish_twist(0.0, 0.0)
            return

        bbox_age = now - self.last_bbox_time

        if bbox_age <= self.bbox_timeout:
            cx = self.smooth_cx
            area = self.smooth_area

            # --- Angular PD-control (keep person centred) ---
            error_x = cx - 0.5
            d_error_x = (error_x - self.prev_error_x) / dt
            self.prev_error_x = error_x

            raw_angular = -self.k_ang * error_x - self.kd_ang * d_error_x
            target_angular = self._apply_deadband_with_min(
                raw_angular, self.center_deadband * self.k_ang, self.min_ang)

            # --- Linear P-control (maintain target distance) ---
            if area >= self.safe_area:
                target_linear = 0.0
            else:
                error_area = self.target_area - area
                raw_linear = self.k_lin * error_area
                target_linear = self._apply_deadband_with_min(
                    raw_linear, 0.0, self.min_lin)

            # Clamp to velocity limits
            target_linear = max(min(target_linear, self.max_lin), -self.max_lin)
            target_angular = max(min(target_angular, self.max_ang), -self.max_ang)

        # Slew rate limiting — smooths acceleration to protect drivetrain
        max_delta_lin = self.accel_lin * dt
        max_delta_ang = self.accel_ang * dt

        smooth_linear = self._apply_slew(
            target_linear, self.last_linear_x, max_delta_lin)
        smooth_angular = self._apply_slew(
            target_angular, self.last_angular_z, max_delta_ang)

        self.publish_twist(smooth_linear, smooth_angular)

    # ── Output ──────────────────────────────────────────────────────────

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

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
