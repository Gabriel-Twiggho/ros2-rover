"""
YOLO Person Detection Node for ROS 2
====================================
This script creates a ROS 2 "node" (a program that can send/receive messages).
It listens for camera images, runs AI detection to find people, and publishes
the results so other nodes (like a robot controller) can use them.

NOTE ON YOLO / GPU REQUIREMENTS:
- This node uses the Ultralytics YOLO Python package installed in the PC venv.
- The package version and model weights DO NOT auto-update unless you manually
  run a pip upgrade (e.g. `pip install -U ultralytics`) or change the model name.
- This protects you from a future YOLO release that might require more VRAM
  than your current GPU (8 GB). To be extra safe, you can pin a known-good
  version in the venv, e.g. `pip install "ultralytics==8.3.0"`.

FLOW:
  Camera --> /camera/image_raw/compressed --> [This Node] --> /person_bbox   (for robot control)
                                                          --> /detections    (standard ROS format)
                                                          --> /yolo_debug    (image with boxes drawn)

To view the debug image:
  ros2 run rqt_image_view rqt_image_view
  Then select '/yolo_debug' from the dropdown.
"""

# =============================================================================
# IMPORTS - Libraries we need
# =============================================================================
import rclpy                          # ROS 2 Python library (the core framework)
from rclpy.node import Node           # Base class for creating ROS 2 nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # Message delivery settings

# ROS 2 Message Types (pre-defined data structures for communication)
from sensor_msgs.msg import CompressedImage, Image  # CompressedImage for input, Image for debug output
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose  # Standard detection format
from std_msgs.msg import Float32MultiArray    # Simple array of numbers

import cv2              # OpenCV - image processing library
import numpy as np      # NumPy - for working with arrays/matrices
from ultralytics import YOLO  # YOLOv8 - the AI object detection model
import torch            # PyTorch - deep learning framework (used to check for GPU)


# =============================================================================
# THE MAIN NODE CLASS
# =============================================================================
class YoloDetectNode(Node):
    """
    A ROS 2 Node that:
    1. Subscribes to compressed camera images
    2. Runs YOLO AI detection to find people
    3. Publishes detection results for other nodes to use
    """
    
    def __init__(self):
        # Initialize the parent Node class with our node's name
        # This name shows up when you run: ros2 node list
        super().__init__('yolo_detect_node')

        # -----------------------------------------------------------------
        # PARAMETERS - Configurable settings (can be changed at runtime)
        # -----------------------------------------------------------------
        # declare_parameter() creates a setting that can be changed when launching
        # Example: ros2 run yolo_detect yolo_detect --ros-args -p model:=yolov8s.pt
        
        self.declare_parameter('model', 'yolov8n.pt')  # Which YOLO model file to use
        # Available models: yolov8n.pt (fast), yolov8s.pt, yolov8m.pt, yolov8l.pt, yolov8x.pt (accurate)
        
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        # 'cuda' = use NVIDIA GPU (fast), 'cpu' = use processor (slower)
        
        self.declare_parameter('conf_thresh', 0.5)
        # Confidence threshold: 0.5 means "only report detections that are 50%+ confident"
        
        # Read the actual values from the parameters
        model_name = self.get_parameter('model').value
        self.device = self.get_parameter('device').value
        self.conf_thresh = self.get_parameter('conf_thresh').value

        # -----------------------------------------------------------------
        # LOAD THE AI MODEL
        # -----------------------------------------------------------------
        self.get_logger().info(f"Loading YOLO model: {model_name} on {self.device}...")
        try:
            # YOLO() downloads the model if needed and loads it into memory
            self.model = YOLO(model_name)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
        
        # -----------------------------------------------------------------
        # QoS (Quality of Service) - How messages are delivered
        # -----------------------------------------------------------------
        # For video, we use "best effort" because:
        # - We only care about the LATEST frame, not old ones
        # - If a frame is lost, just use the next one (don't retry)
        video_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Don't guarantee delivery
            history=HistoryPolicy.KEEP_LAST,            # Only keep recent messages
            depth=1                                      # Only keep 1 message in queue
        )

        # -----------------------------------------------------------------
        # SUBSCRIBER - Listen for incoming camera images
        # -----------------------------------------------------------------
        # create_subscription() tells ROS 2: "Call image_callback() whenever
        # a new message arrives on the '/camera/image_raw/compressed' topic"
        self.subscription = self.create_subscription(
            CompressedImage,                    # Message type we expect
            '/camera/image_raw/compressed',     # Topic name to listen to
            self.image_callback,                # Function to call when message arrives
            video_qos                           # Delivery settings
        )
        
        # -----------------------------------------------------------------
        # PUBLISHERS - Send out detection results
        # -----------------------------------------------------------------
        # Publisher 1: Full detection details (standard ROS format)
        # Use this if you want to visualize in RViz or use with other ROS tools
        self.detection_pub = self.create_publisher(
            Detection2DArray,   # Message type
            '/detections',      # Topic name
            10                  # Queue size (buffer up to 10 messages)
        )
            
        # Publisher 2: Simplified data for follow-me robot logic
        # Just 4 numbers: [center_x, center_y, area, confidence]
        # This is easier to use in a simple robot controller
        self.person_bbox_pub = self.create_publisher(
            Float32MultiArray, 
            '/person_bbox', 
            10
        )
        
        # Publisher 3: Debug image with bounding boxes drawn
        # View this with: ros2 run rqt_image_view rqt_image_view
        # Then select /yolo_debug from the dropdown
        self.debug_pub = self.create_publisher(
            Image,
            '/yolo_debug',
            10
        )

        self.get_logger().info("YOLO Detect Node is ready.")


    # =========================================================================
    # CALLBACK - This function runs every time a new camera frame arrives
    # =========================================================================
    def image_callback(self, msg):
        """
        Called automatically whenever a new image message arrives.
        
        Args:
            msg: A CompressedImage message containing JPEG data from the camera
        """
        try:
            # -----------------------------------------------------------------
            # STEP 1: Convert compressed image to OpenCV format
            # -----------------------------------------------------------------
            # msg.data contains raw JPEG bytes (like a .jpg file in memory)
            # We need to convert it to a numpy array that OpenCV can work with
            
            # frombuffer() interprets the bytes as an array of uint8 (0-255 values)
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # imdecode() decompresses the JPEG into a full image
            # IMREAD_COLOR means we want a color image (BGR format, not grayscale)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            #the image comes in upside down so lets flip it
            #cv_image = cv2.flip(cv_image, -1)
            
            # Safety check - make sure decoding worked
            if cv_image is None:
                self.get_logger().error("Failed to decode image!")
                return
            
            # -----------------------------------------------------------------
            # STEP 2: Run YOLO AI Detection
            # -----------------------------------------------------------------
            # self.model() runs the neural network on the image
            # - verbose=False: Don't print detection info to console
            # - conf=self.conf_thresh: Only return detections above this confidence
            # - classes=0: Only detect class 0 which is "person" in COCO dataset
            #   (YOLO can detect 80 different things: person, car, dog, etc.)
            results = self.model(cv_image, verbose=False, conf=self.conf_thresh, classes=0)
            
            # -----------------------------------------------------------------
            # STEP 3: Process the detection results
            # -----------------------------------------------------------------
            # Create message objects to hold our results
            det_array = Detection2DArray()      # Will hold all detections
            det_array.header = msg.header       # Copy timestamp from original image
            
            # Variables to track the biggest person (for follow-me logic)
            max_area = 0
            best_person = None

            # Loop through all detection results
            # (usually just 1 result, but the API returns a list)
            for r in results:
                boxes = r.boxes  # Get all detected bounding boxes
                
                # Loop through each detected person
                for box in boxes:
                    # Extract bounding box coordinates
                    # xyxy format: [x1, y1, x2, y2] = [left, top, right, bottom]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Get confidence score (0.0 to 1.0, higher = more confident)
                    conf = float(box.conf[0].cpu().numpy())
                    
                    # Calculate useful values from the bounding box
                    width = x2 - x1                    # Box width in pixels
                    height = y2 - y1                   # Box height in pixels
                    cx = x1 + (width / 2)              # Center X coordinate
                    cy = y1 + (height / 2)             # Center Y coordinate
                    area = width * height              # Box area in pixels²

                    # --- Build the standard ROS Detection2D message ---
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # Add hypothesis (what we think this detection is)
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = "person"  # Label
                    hypothesis.hypothesis.score = conf         # Confidence
                    detection.results.append(hypothesis)
                    
                    # Add bounding box info
                    detection.bbox.center.position.x = float(cx)
                    detection.bbox.center.position.y = float(cy)
                    detection.bbox.size_x = float(width)
                    detection.bbox.size_y = float(height)
                    
                    det_array.detections.append(detection)

                    # --- Track the largest person for follow-me logic ---
                    # We assume the biggest bounding box = the person to follow
                    if area > max_area:
                        max_area = area
                        
                        # Normalize coordinates to 0.0 - 1.0 range
                        # This makes values independent of camera resolution
                        # (works the same for 640x480 or 1920x1080)
                        img_h, img_w, _ = cv_image.shape  # Get image dimensions
                        norm_cx = cx / img_w      # 0.0 = left edge, 1.0 = right edge
                        norm_cy = cy / img_h      # 0.0 = top edge, 1.0 = bottom edge
                        norm_area = area / (img_w * img_h)  # 0.0 = tiny, 1.0 = fills frame
                        
                        best_person = [norm_cx, norm_cy, norm_area, conf]

            # -----------------------------------------------------------------
            # STEP 4: Publish the results
            # -----------------------------------------------------------------
            # Always publish the full detection array (even if empty)
            self.detection_pub.publish(det_array)

            # Only publish to /person_bbox if we found at least one person
            if best_person:
                bbox_msg = Float32MultiArray()
                bbox_msg.data = best_person  # [cx, cy, area, confidence]
                self.person_bbox_pub.publish(bbox_msg)

            # -----------------------------------------------------------------
            # STEP 5: Publish debug image with bounding boxes drawn
            # -----------------------------------------------------------------
            # YOLO's .plot() method draws all detections on the image for us!
            # This is useful for debugging - you can see what the AI sees
            annotated_frame = results[0].plot()
            
            # Convert OpenCV image (numpy array) to ROS Image message
            # We do this manually to avoid needing cv_bridge (Windows compatibility)
            debug_msg = Image()
            debug_msg.header = msg.header                    # Copy timestamp
            debug_msg.height = annotated_frame.shape[0]      # Image height in pixels
            debug_msg.width = annotated_frame.shape[1]       # Image width in pixels
            debug_msg.encoding = 'bgr8'                      # Color format (Blue-Green-Red, 8 bits each)
            debug_msg.is_bigendian = 0                       # Byte order (0 = little endian, standard for x86)
            debug_msg.step = annotated_frame.shape[1] * 3    # Bytes per row (width * 3 channels)
            debug_msg.data = annotated_frame.tobytes()       # The actual pixel data
            
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            # Log any errors (helpful for debugging)
            self.get_logger().error(f"Error processing image: {e}")


# =============================================================================
# MAIN FUNCTION - Entry point when script is run
# =============================================================================
def main(args=None):
    """
    Main function that starts the ROS 2 node.
    Called when you run: ros2 run yolo_detect yolo_detect_node
    """
    # Initialize the ROS 2 Python library
    rclpy.init(args=args)
    
    # Create an instance of our node
    node = YoloDetectNode()
    
    try:
        # spin() keeps the node running, processing callbacks until interrupted
        # This is a blocking call - it won't return until the node shuts down
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User pressed Ctrl+C - this is a normal way to stop
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


# This is Python's way of saying "run main() if this script is executed directly"
# (as opposed to being imported as a module)
if __name__ == '__main__':
    main()
