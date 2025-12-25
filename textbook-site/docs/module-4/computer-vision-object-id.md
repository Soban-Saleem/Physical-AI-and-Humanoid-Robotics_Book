# Computer Vision for Object Identification

Computer vision is a fundamental component of Vision-Language-Action (VLA) systems, enabling humanoid robots to perceive and understand their environment. In this section, we'll explore how to implement computer vision systems for object identification, scene understanding, and visual perception in the context of Physical AI & Humanoid Robotics.

## Learning Objectives

By the end of this section, you will be able to:
- Implement object detection and recognition systems for robotics
- Integrate computer vision with ROS 2 messaging
- Process visual data in real-time for robotic applications
- Implement 3D object pose estimation from 2D images
- Connect vision systems with language understanding and action planning
- Optimize computer vision pipelines for humanoid robotics
- Validate vision system outputs for safety and accuracy

## Introduction to Computer Vision in Robotics

Computer vision in robotics goes beyond simple image processing to encompass:
- **Object Detection**: Identifying and localizing objects in the environment
- **Object Recognition**: Classifying objects into known categories
- **Pose Estimation**: Determining the 3D position and orientation of objects
- **Scene Understanding**: Interpreting the spatial relationships between objects
- **Visual Servoing**: Using visual feedback to control robot motion
- **SLAM**: Simultaneous Localization and Mapping using visual data

For humanoid robots, computer vision must be robust enough to handle:
- Varying lighting conditions
- Different viewing angles and distances
- Occlusions and cluttered environments
- Dynamic scenes with moving objects
- Human-robot interaction scenarios

## Setting Up Computer Vision in Isaac ROS

Isaac ROS provides optimized computer vision capabilities specifically designed for robotics applications:

### Isaac ROS Vision Packages

- **Isaac ROS Apriltag**: GPU-accelerated fiducial marker detection
- **Isaac ROS Stereo Dense Reconstruction**: 3D reconstruction from stereo cameras
- **Isaac ROS Visual SLAM**: GPU-accelerated SLAM algorithms
- **Isaac ROS DNN Inference**: GPU-accelerated deep neural network inference
- **Isaac ROS Image Pipeline**: Optimized image acquisition and preprocessing

### Installation

```bash
# Install Isaac ROS vision packages
sudo apt update
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-stereo-dense-reconstruction
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-dnn-inference
sudo apt install ros-humble-isaac-ros-image-pipeline
```

## Object Detection Implementation

### Basic Object Detection Node

```python
# object_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class IsaacROSObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Subscribe to camera image and camera info
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10)
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        
        # Publish object detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10)
        
        # Initialize CV bridge and YOLO model
        self.bridge = CvBridge()
        self.camera_info = None
        self.intrinsics = None
        
        # Load YOLO model (using a model trained on robotics objects)
        try:
            # For this example, we'll use a pre-trained COCO model
            # In practice, you'd use a model trained on robotics-specific data
            self.model = YOLO('yolov8n.pt')  # Use smaller model for faster inference
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None
        
        # Detection parameters
        self.confidence_threshold = 0.5
        self.class_names = self.model.names if self.model else {}
        
        self.get_logger().info('Isaac ROS Object Detector initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsics for 3D reconstruction"""
        self.camera_info = msg
        self.intrinsics = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],  # fx, 0, cx
            [msg.k[3], msg.k[4], msg.k[5]],  # 0, fy, cy
            [msg.k[6], msg.k[7], msg.k[8]]   # 0, 0, 1
        ])

    def image_callback(self, msg):
        """Process image and detect objects"""
        if not self.model:
            return
            
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run object detection
            results = self.model(cv_image, conf=self.confidence_threshold)
            
            # Convert results to ROS format
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            for result in results:
                for detection in result.boxes:
                    if detection.conf >= self.confidence_threshold:
                        # Create detection message
                        detection_msg = Detection2D()
                        detection_msg.header = msg.header
                        
                        # Set bounding box
                        bbox = detection.xywh[0]  # x, y, width, height
                        detection_msg.bbox.center.x = float(bbox[0])
                        detection_msg.bbox.center.y = float(bbox[1])
                        detection_msg.bbox.size_x = float(bbox[2])
                        detection_msg.bbox.size_y = float(bbox[3])
                        
                        # Add hypothesis with confidence
                        hypothesis = ObjectHypothesisWithPose()
                        class_id = int(detection.cls[0])
                        class_name = self.class_names[class_id] if class_id in self.class_names else f"unknown_{class_id}"
                        
                        hypothesis.id = class_name
                        hypothesis.score = float(detection.conf[0])
                        
                        detection_msg.results.append(hypothesis)
                        detections_msg.detections.append(detection_msg)
            
            # Publish detections
            self.detection_publisher.publish(detections_msg)
            
            self.get_logger().info(f'Detected {len(detections_msg.detections)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    detector = IsaacROSObjectDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Shutting down object detector...')
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3D Object Pose Estimation

For humanoid robots, understanding the 3D position and orientation of objects is crucial for manipulation:

### Pose Estimation Node

```python
# pose_estimator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point, Quaternion

class IsaacROSPoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        
        # Subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10)
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        
        # Publishers
        self.pose_publisher = self.create_publisher(
            PoseArray,
            '/object_poses',
            10)
        
        # Initialize components
        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_depth = None
        
        self.get_logger().info('Isaac ROS Pose Estimator initialized')

    def camera_info_callback(self, msg):
        """Store camera calibration information"""
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.latest_depth = depth_image
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def image_callback(self, msg):
        """Process image and estimate object poses"""
        if self.camera_matrix is None or self.latest_depth is None:
            return
            
        try:
            # Convert image to OpenCV format
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # First, detect objects in the image
            # For this example, we'll use a simple color-based detection
            # In practice, you'd use a more sophisticated object detector
            
            # Convert to HSV for color-based detection
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            
            # Define color range for a specific object (e.g., red ball)
            lower_red = np.array([0, 50, 50])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            
            lower_red = np.array([170, 50, 50])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)
            
            mask = mask1 + mask2
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create pose array
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            for contour in contours:
                # Filter small contours
                if cv2.contourArea(contour) < 100:
                    continue
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Get depth at the center of the object
                depth_value = self.latest_depth[center_y, center_x]
                
                if np.isnan(depth_value) or depth_value == 0:
                    continue  # Skip if depth is invalid
                
                # Convert pixel coordinates to 3D world coordinates
                # Using pinhole camera model: X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]
                
                world_x = (center_x - cx) * depth_value / fx
                world_y = (center_y - cy) * depth_value / fy
                world_z = depth_value  # Depth value is the Z coordinate
                
                # Create pose
                pose = Pose()
                pose.position.x = world_x
                pose.position.y = world_y
                pose.position.z = world_z
                
                # For now, set orientation to identity (no rotation)
                pose.orientation.w = 1.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                
                pose_array.poses.append(pose)
            
            # Publish poses
            self.pose_publisher.publish(pose_array)
            
            self.get_logger().info(f'Estimated poses for {len(pose_array.poses)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Error in pose estimation: {e}')

def main(args=None):
    rclpy.init(args=args)
    pose_estimator = IsaacROSPoseEstimator()
    
    try:
        rclpy.spin(pose_estimator)
    except KeyboardInterrupt:
        pose_estimator.get_logger().info('Shutting down pose estimator...')
    finally:
        pose_estimator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Isaac ROS DNN Inference Integration

For more advanced computer vision, we can integrate Isaac ROS DNN inference:

```python
# dnn_vision_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import torch

class IsaacROSDNNVision(Node):
    def __init__(self):
        super().__init__('dnn_vision_pipeline')
        
        # Subscribe to camera input
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10)
        
        # Publish detection results
        self.result_publisher = self.create_publisher(
            String,  # In practice, use a more structured message type
            '/dnn_vision_results',
            10)
        
        # Initialize components
        self.bridge = CvBridge()
        
        # Initialize neural network model
        # In a real implementation, this would be an Isaac ROS DNN node
        # For this example, we'll simulate the DNN processing
        self.model_loaded = True
        self.get_logger().info('Isaac ROS DNN Vision Pipeline initialized')

    def image_callback(self, msg):
        """Process image using DNN inference"""
        try:
            # Convert ROS image to tensor format for DNN processing
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # In a real Isaac ROS implementation, this would be processed by:
            # - Isaac ROS DNN inference nodes
            # - GPU-accelerated processing
            # - TensorRT optimization
            
            # Simulate DNN processing
            results = self.simulate_dnn_processing(cv_image)
            
            # Publish results
            result_msg = String()
            result_msg.data = str(results)
            self.result_publisher.publish(result_msg)
            
            self.get_logger().info(f'DNN vision results: {results}')
            
        except Exception as e:
            self.get_logger().error(f'Error in DNN vision processing: {e}')

    def simulate_dnn_processing(self, image):
        """Simulate DNN processing results"""
        # This is a placeholder - in real implementation, 
        # this would interface with Isaac ROS DNN nodes
        height, width = image.shape[:2]
        
        # Simulate detection of common objects in robotics context
        simulated_detections = [
            {
                "class": "person",
                "confidence": 0.92,
                "bbox": [int(width*0.3), int(height*0.2), int(width*0.4), int(height*0.6)],
                "center_3d": [1.2, 0.5, 0.0]  # Simulated 3D coordinates
            },
            {
                "class": "cup",
                "confidence": 0.87,
                "bbox": [int(width*0.7), int(height*0.5), int(width*0.8), int(height*0.6)],
                "center_3d": [0.8, -0.3, 0.8]  # Simulated 3D coordinates
            }
        ]
        
        return simulated_detections

def main(args=None):
    rclpy.init(args=args)
    dnn_vision = IsaacROSDNNVision()
    
    try:
        rclpy.spin(dnn_vision)
    except KeyboardInterrupt:
        dnn_vision.get_logger().info('Shutting down DNN vision pipeline...')
    finally:
        dnn_vision.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visual Servoing

Visual servoing uses visual feedback to control robot motion:

```python
# visual_servoing.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np

class VisualServoingController(Node):
    def __init__(self):
        super().__init__('visual_servoing_controller')
        
        # Subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10)
        
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10)
        
        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # Initialize components
        self.bridge = CvBridge()
        self.target_object = "cup"  # Object to track
        self.tracking_active = False
        self.current_target_center = None
        self.image_center = None
        
        # Control parameters
        self.linear_scale = 0.2  # Scale for linear velocity
        self.angular_scale = 0.5  # Scale for angular velocity
        self.threshold = 0.1  # Threshold for centering (normalized coordinates)
        
        self.get_logger().info('Visual Servoing Controller initialized')

    def detection_callback(self, msg):
        """Process object detections for visual servoing"""
        # Find the target object in detections
        target_detection = None
        for detection in msg.detections:
            if detection.results and len(detection.results) > 0:
                if detection.results[0].id == self.target_object:
                    target_detection = detection
                    break
        
        if target_detection:
            # Get image dimensions from the header or assume standard size
            img_width = 640  # These should come from camera info in real implementation
            img_height = 480
            
            # Calculate center of the detected object
            obj_center_x = target_detection.bbox.center.x
            obj_center_y = target_detection.bbox.center.y
            
            # Normalize to image center
            normalized_x = (obj_center_x - img_width/2) / (img_width/2)
            normalized_y = (obj_center_y - img_height/2) / (img_height/2)
            
            self.current_target_center = (normalized_x, normalized_y)
            self.tracking_active = True
        else:
            self.tracking_active = False
            self.current_target_center = None

    def image_callback(self, msg):
        """Process image to update image center"""
        # In a real implementation, this would come from camera info
        # For this example, we'll assume standard image size
        self.image_center = (320, 240)  # Center of 640x480 image

    def control_loop(self):
        """Main control loop for visual servoing"""
        if not self.tracking_active or self.current_target_center is None:
            # Stop the robot if not tracking
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            return
        
        # Get normalized coordinates of target
        target_x, target_y = self.current_target_center
        
        # Create command to center the target
        cmd_vel = Twist()
        
        # If target is not centered horizontally, rotate
        if abs(target_x) > self.threshold:
            cmd_vel.angular.z = -self.angular_scale * target_x  # Negative to correct direction
        
        # If target is not centered vertically, move forward/backward
        if abs(target_y) > self.threshold:
            cmd_vel.linear.x = -self.linear_scale * target_y  # Negative to correct direction
        
        # If target is centered, move forward to approach
        if abs(target_x) <= self.threshold and abs(target_y) <= self.threshold:
            cmd_vel.linear.x = 0.3  # Move forward to approach target
        
        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)
        
        self.get_logger().info(f'Visual servoing: angular.z={cmd_vel.angular.z:.2f}, linear.x={cmd_vel.linear.x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    servo_controller = VisualServoingController()
    
    # Create a timer for the control loop
    servo_controller.control_timer = servo_controller.create_timer(0.1, servo_controller.control_loop)
    
    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        servo_controller.get_logger().info('Shutting down visual servoing controller...')
    finally:
        # Stop the robot before shutting down
        stop_cmd = Twist()
        servo_controller.cmd_vel_publisher.publish(stop_cmd)
        
        servo_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Language Understanding

Connecting vision systems with language understanding enables complex VLA behaviors:

```python
# vision_language_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import json

class VisionLanguageIntegrator(Node):
    def __init__(self):
        super().__init__('vision_language_integrator')
        
        # Subscriptions
        self.vision_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.vision_callback,
            10)
        
        self.language_subscription = self.create_subscription(
            String,
            '/parsed_intents',
            self.language_callback,
            10)
        
        # Publishers
        self.action_publisher = self.create_publisher(
            String,  # In practice, use a structured action message
            '/integrated_actions',
            10)
        
        self.response_publisher = self.create_publisher(
            String,
            '/robot_responses',
            10)
        
        # Initialize components
        self.bridge = CvBridge()
        self.current_objects = {}
        self.pending_language_command = None
        
        self.get_logger().info('Vision-Language Integrator initialized')

    def vision_callback(self, msg):
        """Update knowledge of visible objects"""
        for detection in msg.detections:
            if detection.results and len(detection.results) > 0:
                object_class = detection.results[0].id
                confidence = detection.results[0].score
                bbox = detection.bbox
                
                # Store object information
                self.current_objects[object_class] = {
                    'bbox': {
                        'center_x': bbox.center.x,
                        'center_y': bbox.center.y,
                        'size_x': bbox.size_x,
                        'size_y': bbox.size_y
                    },
                    'confidence': confidence,
                    'timestamp': self.get_clock().now().nanoseconds
                }
        
        self.get_logger().info(f'Updated object knowledge: {list(self.current_objects.keys())}')

    def language_callback(self, msg):
        """Process language commands and integrate with vision"""
        try:
            # Parse the intent message (format: "INTENT_TYPE|{entities_json}")
            parts = msg.data.split('|', 1)
            if len(parts) != 2:
                return
                
            intent_type = parts[0]
            entities = json.loads(parts[1])
            
            # Handle commands that require vision integration
            if intent_type == 'PICK_UP_OBJECT':
                object_name = entities.get('object', '').lower()
                self.execute_pick_up_command(object_name)
            elif intent_type == 'NAVIGATE_TO_LOCATION' and 'object' in entities:
                object_name = entities['object'].lower()
                self.execute_navigate_to_object_command(object_name)
            else:
                # For other commands, just acknowledge
                response_msg = String()
                response_msg.data = f"Received command: {intent_type}"
                self.response_publisher.publish(response_msg)
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in language message: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing language command: {e}')

    def execute_pick_up_command(self, object_name):
        """Execute pick up command using vision information"""
        # Check if the requested object is visible
        if object_name in self.current_objects:
            object_info = self.current_objects[object_name]
            confidence = object_info['confidence']
            
            if confidence > 0.5:  # Threshold for reliable detection
                # Determine approach strategy based on object position
                center_x = object_info['bbox']['center_x']
                img_width = 640  # Should come from camera info
                
                # Normalize object position
                normalized_x = (center_x - img_width/2) / (img_width/2)
                
                # Create action sequence
                action_sequence = [
                    {
                        'action_type': 'align_with_object',
                        'parameters': {
                            'horizontal_offset': normalized_x,
                            'object_name': object_name
                        },
                        'description': f'Align with {object_name} at position {normalized_x}'
                    },
                    {
                        'action_type': 'approach_object',
                        'parameters': {
                            'distance': 0.3,  # 30cm from object
                            'object_name': object_name
                        },
                        'description': f'Approach {object_name}'
                    },
                    {
                        'action_type': 'grasp_object',
                        'parameters': {
                            'object_name': object_name
                        },
                        'description': f'Grasp {object_name}'
                    }
                ]
                
                # Publish action sequence
                action_msg = String()
                action_msg.data = json.dumps({
                    'sequence': action_sequence,
                    'target_object': object_name
                })
                self.action_publisher.publish(action_msg)
                
                response_msg = String()
                response_msg.data = f"Attempting to pick up {object_name}"
                self.response_publisher.publish(response_msg)
            else:
                # Object is visible but confidence is low
                response_msg = String()
                response_msg.data = f"I see {object_name} but the detection is not confident enough to pick it up"
                self.response_publisher.publish(response_msg)
        else:
            # Object not visible
            response_msg = String()
            response_msg.data = f"I cannot see {object_name} in my current view. Could you move it into view or reposition the robot?"
            self.response_publisher.publish(response_msg)

    def execute_navigate_to_object_command(self, object_name):
        """Execute navigation to object command"""
        if object_name in self.current_objects:
            object_info = self.current_objects[object_name]
            confidence = object_info['confidence']
            
            if confidence > 0.5:
                # Create navigation action
                action = {
                    'action_type': 'navigate_to_object',
                    'parameters': {
                        'object_name': object_name,
                        'target_distance': 1.0  # 1 meter from object
                    },
                    'description': f'Navigate to {object_name}'
                }
                
                action_msg = String()
                action_msg.data = json.dumps([action])
                self.action_publisher.publish(action_msg)
                
                response_msg = String()
                response_msg.data = f"Navigating to {object_name}"
                self.response_publisher.publish(response_msg)
            else:
                response_msg = String()
                response_msg.data = f"I see {object_name} but the detection is not confident enough to navigate to it"
                self.response_publisher.publish(response_msg)
        else:
            response_msg = String()
            response_msg.data = f"I cannot see {object_name} to navigate to it"
            self.response_publisher.publish(response_msg)

def main(args=None):
    rclpy.init(args=args)
    vli = VisionLanguageIntegrator()
    
    try:
        rclpy.spin(vli)
    except KeyboardInterrupt:
        vli.get_logger().info('Shutting down vision-language integrator...')
    finally:
        vli.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization for Computer Vision

### Efficient Processing Pipelines

```python
# optimized_vision_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import threading
from queue import Queue
import time
import numpy as np

class OptimizedVisionPipeline(Node):
    def __init__(self):
        super().__init__('optimized_vision_pipeline')
        
        # Create queues for efficient processing
        self.input_queue = Queue(maxsize=2)  # Only keep latest 2 frames
        self.output_queue = Queue(maxsize=2)
        
        # Subscription with low QoS for performance
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.optimized_image_callback,
            1)  # Low queue size to avoid old frame processing
        
        # Publisher
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/optimized_detections',
            1)
        
        # Initialize components
        self.bridge = CvBridge()
        self.processing_thread = threading.Thread(target=self.processing_loop)
        self.processing_thread.daemon = True
        self.running = True
        
        # Processing parameters
        self.process_every_n_frames = 3  # Process every 3rd frame to reduce CPU load
        self.frame_counter = 0
        
        # Start processing thread
        self.processing_thread.start()
        
        self.get_logger().info('Optimized Vision Pipeline initialized')

    def optimized_image_callback(self, msg):
        """Optimized image callback that uses queuing"""
        try:
            # Only process every Nth frame to reduce load
            self.frame_counter += 1
            if self.frame_counter % self.process_every_n_frames != 0:
                return  # Skip this frame
            
            # Only add to queue if there's space (don't block)
            if not self.input_queue.full():
                self.input_queue.put((msg.header, msg))
            else:
                # Queue full, drop oldest frame
                try:
                    self.input_queue.get_nowait()
                    self.input_queue.put((msg.header, msg))
                except:
                    pass  # Queue might have been emptied by another thread
        except Exception as e:
            self.get_logger().error(f'Error in optimized callback: {e}')

    def processing_loop(self):
        """Background processing loop"""
        while self.running:
            try:
                # Get image from queue (with timeout to allow checking running flag)
                header, img_msg = self.input_queue.get(timeout=0.1)
                
                # Process the image (in a real implementation, this would run detection)
                detections = self.process_image(img_msg)
                
                if detections and not self.output_queue.full():
                    self.output_queue.put((header, detections))
                    
                    # Publish from main thread via timer
                    self.timer = self.create_timer(0.01, self.publish_results)
                    
            except Exception as e:
                if "timeout" not in str(e).lower():  # Don't log timeouts
                    self.get_logger().error(f'Error in processing loop: {e}')

    def process_image(self, img_msg):
        """Process image and return detections (simplified for example)"""
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            
            # In a real implementation, this would run object detection
            # For this example, we'll return a dummy detection
            height, width = cv_image.shape[:2]
            
            # Create a dummy detection
            detections = Detection2DArray()
            detections.header = img_msg.header
            
            # Add one dummy detection
            detection = Detection2D()
            detection.header = img_msg.header
            detection.bbox.center.x = width / 2
            detection.bbox.center.y = height / 2
            detection.bbox.size_x = width / 4
            detection.bbox.size_y = height / 4
            
            from vision_msgs.msg import ObjectHypothesisWithPose
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = "dummy_object"
            hypothesis.score = 0.9
            detection.results.append(hypothesis)
            
            detections.detections.append(detection)
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            return None

    def publish_results(self):
        """Publish results from the output queue"""
        try:
            if not self.output_queue.empty():
                header, detections = self.output_queue.get_nowait()
                if detections:
                    self.detection_publisher.publish(detections)
        except Exception:
            pass  # Queue might be empty, which is fine

    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    opt_vision = OptimizedVisionPipeline()
    
    try:
        rclpy.spin(opt_vision)
    except KeyboardInterrupt:
        opt_vision.get_logger().info('Shutting down optimized vision pipeline...')
    finally:
        opt_vision.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Computer Vision in Robotics

### 1. Real-time Performance
- Process only essential frames to maintain performance
- Use appropriate model sizes for target hardware
- Implement efficient data structures and algorithms
- Consider hardware acceleration (GPU, Jetson, etc.)

### 2. Robustness
- Handle varying lighting conditions
- Account for different viewing angles
- Implement fallback mechanisms when detection fails
- Validate detection results before acting on them

### 3. Safety
- Verify object poses are physically plausible
- Check that manipulation actions are safe
- Implement limits on robot motion based on vision results
- Include human safety considerations

### 4. Integration
- Ensure vision system outputs are compatible with action planning
- Maintain temporal consistency in object tracking
- Synchronize vision processing with robot control loops
- Provide uncertainty estimates with detections

## Troubleshooting Common Issues

### 1. Detection Performance
- **Issue**: Slow detection performance
- **Solution**: Use smaller models, process fewer frames, optimize image resolution

### 2. False Positives
- **Issue**: Detecting objects that aren't there
- **Solution**: Adjust confidence thresholds, use domain-specific training data

### 3. Missed Detections
- **Issue**: Failing to detect objects that are present
- **Solution**: Lower confidence thresholds, improve lighting, use better models

### 4. Integration Problems
- **Issue**: Vision system not communicating properly with action system
- **Solution**: Check message formats, verify topic names, ensure proper timestamps

## Summary

In this section, we've explored computer vision systems for object identification in the context of humanoid robotics. We've covered:

- Basic object detection using Isaac ROS packages
- 3D pose estimation from 2D images and depth data
- Isaac ROS DNN inference integration for advanced vision
- Visual servoing for closed-loop visual control
- Integration with language understanding for VLA systems
- Performance optimization techniques for real-time vision

Computer vision is a critical component of Vision-Language-Action systems, enabling robots to perceive and understand their environment. Proper implementation of these systems is essential for the success of the Physical AI & Humanoid Robotics textbook project.

In the next section, we'll explore manipulation capabilities for humanoid robots, which builds upon the vision systems we've implemented here.