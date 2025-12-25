# AI Perception Systems for Humanoid Robotics

AI perception systems form the sensory foundation for humanoid robots, enabling them to understand and interact with their environment. This section covers how to implement perception systems using NVIDIA Isaac Sim and Isaac ROS packages, with a focus on embodied AI for humanoid robotics applications.

## Learning Objectives

By the end of this section, you will be able to:
- Implement computer vision systems for humanoid robotics using Isaac ROS packages
- Design multimodal perception combining vision, audio, and tactile sensing
- Integrate AI models with real-time perception pipelines
- Optimize perception systems for humanoid-specific tasks (grasping, navigation, interaction)
- Implement sensor fusion for robust perception in challenging environments
- Validate perception system accuracy and reliability in simulation

## Introduction to AI Perception in Humanoid Robotics

Humanoid robots require sophisticated perception systems that go beyond traditional mobile robots. Unlike wheeled robots that primarily need to navigate around obstacles, humanoid robots must:

- Recognize and manipulate objects with human-like hands
- Navigate complex environments designed for humans (stairs, narrow passages, furniture)
- Interpret human gestures, facial expressions, and voice commands
- Understand spatial relationships in 3D environments
- Maintain balance while perceiving the world around them

### Key Perception Capabilities for Humanoid Robots

1. **Visual Perception**: Object recognition, scene understanding, visual navigation
2. **Audio Perception**: Voice command recognition, sound localization, environmental audio understanding
3. **Tactile Perception**: Force/torque sensing for manipulation and balance
4. **Multimodal Fusion**: Combining multiple sensory inputs for robust perception

## Visual Perception Systems

### Object Recognition and Classification

Humanoid robots need to recognize and classify objects in their environment to interact with them appropriately:

```python
# visual_perceptor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import numpy as np
import torch
import torchvision.transforms as T

class IsaacROSVisualPerceptor(Node):
    def __init__(self):
        super().__init__('isaac_ros_visual_perceptor')
        
        # Initialize computer vision model
        self.initialize_cv_model()
        
        # Subscribe to camera topics
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
        
        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.get_logger().info('Isaac ROS Visual Perceptor initialized')

    def initialize_cv_model(self):
        """Initialize the computer vision model"""
        # Using a pre-trained model for demonstration
        # In practice, this would be a model trained on Isaac Sim synthetic data
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()
        
        # Move to GPU if available
        if torch.cuda.is_available():
            self.model.cuda()
            self.get_logger().info('Model loaded on GPU')
        else:
            self.get_logger().info('Model loaded on CPU')

    def camera_info_callback(self, msg):
        """Store camera intrinsics for 3D reconstruction"""
        self.camera_intrinsics = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ])

    def image_callback(self, msg):
        """Process image and detect objects using AI model"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run inference
            results = self.model(cv_image)
            
            # Convert results to ROS format
            detections = Detection2DArray()
            detections.header = msg.header
            
            for *xyxy, conf, cls in results.xyxy[0].tolist():
                if conf > 0.5:  # Confidence threshold
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # Convert bounding box coordinates
                    x_center = (xyxy[0] + xyxy[2]) / 2.0
                    y_center = (xyxy[1] + xyxy[3]) / 2.0
                    width = xyxy[2] - xyxy[0]
                    height = xyxy[3] - xyxy[1]
                    
                    detection.bbox.center.x = x_center
                    detection.bbox.center.y = y_center
                    detection.bbox.size_x = width
                    detection.bbox.size_y = height
                    
                    # Add hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = int(cls)
                    hypothesis.score = conf
                    detection.results.append(hypothesis)
                    
                    detections.detections.append(detection)
            
            # Publish detections
            self.detection_publisher.publish(detections)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    visual_perceptor = IsaacROSVisualPerceptor()
    
    try:
        rclpy.spin(visual_perceptor)
    except KeyboardInterrupt:
        visual_perceptor.get_logger().info('Shutting down visual perceptor...')
    finally:
        visual_perceptor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3D Object Detection and Pose Estimation

For humanoid robots that need to manipulate objects, 3D pose estimation is crucial:

```python
# pose_estimator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d

class IsaacROSPoseEstimator(Node):
    def __init__(self):
        super().__init__('isaac_ros_pose_estimator')
        
        # Subscribe to RGB and depth images
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.rgb_callback,
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
        
        # Publish 3D object poses
        self.pose_publisher = self.create_publisher(
            Detection3DArray,
            '/object_poses',
            10)
        
        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.latest_depth = None
        self.get_logger().info('Isaac ROS Pose Estimator initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsics for 3D reconstruction"""
        self.camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=msg.width,
            height=msg.height,
            fx=msg.k[0],
            fy=msg.k[4],
            cx=msg.k[2],
            cy=msg.k[5]
        )

    def depth_callback(self, msg):
        """Store the latest depth image"""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def rgb_callback(self, msg):
        """Process RGB image to estimate 3D poses of detected objects"""
        if self.latest_depth is None or self.camera_intrinsics is None:
            return
            
        # Convert images to numpy arrays
        rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth_image = self.latest_depth
        
        # In a real implementation, this would:
        # 1. Use the visual perceptor to get 2D object detections
        # 2. Extract the corresponding regions from the depth image
        # 3. Estimate 3D poses using PnP or similar algorithms
        
        # For demonstration, we'll simulate the process
        detections_3d = Detection3DArray()
        detections_3d.header = msg.header
        
        # Simulated detection - in reality, this would come from the 2D detector
        # and be matched with depth information
        for i in range(3):  # Simulate 3 detected objects
            detection_3d = Detection3D()
            
            # Simulated pose (in reality, this would be computed from depth + 2D bbox)
            pose = Pose()
            pose.position.x = np.random.uniform(-1, 1)
            pose.position.y = np.random.uniform(-1, 1)
            pose.position.z = np.random.uniform(0.5, 2.0)
            
            # Simple orientation (identity quaternion)
            pose.orientation.w = 1.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            
            detection_3d.results.append(ObjectHypothesisWithPose())
            detection_3d.bbox.center = pose
            detection_3d.bbox.size.x = 0.1  # Estimated size
            detection_3d.bbox.size.y = 0.1
            detection_3d.bbox.size.z = 0.1
            
            detections_3d.detections.append(detection_3d)
        
        self.pose_publisher.publish(detections_3d)

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

## Audio Perception Systems

Humanoid robots need to process audio for voice commands and environmental awareness:

```python
# audio_perceptor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
import numpy as np
import librosa
import speech_recognition as sr

class IsaacROSAudioPerceptor(Node):
    def __init__(self):
        super().__init__('isaac_ros_audio_perceptor')
        
        # Subscribe to audio input
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10)
        
        # Publish recognized commands
        self.command_publisher = self.create_publisher(
            String,
            '/recognized_commands',
            10)
        
        # Initialize audio processing components
        self.recognizer = sr.Recognizer()
        self.get_logger().info('Isaac ROS Audio Perceptor initialized')

    def audio_callback(self, msg):
        """Process audio data and recognize speech commands"""
        try:
            # Convert AudioData message to audio format for processing
            # This is a simplified example - actual implementation would depend on audio format
            audio_bytes = bytes(msg.data)
            
            # Process audio to extract commands
            command = self.process_audio_command(audio_bytes)
            
            if command:
                # Publish recognized command
                cmd_msg = String()
                cmd_msg.data = command
                self.command_publisher.publish(cmd_msg)
                
                self.get_logger().info(f'Recognized command: {command}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def process_audio_command(self, audio_bytes):
        """Process audio bytes and extract commands"""
        # In a real implementation, this would use:
        # 1. Speech recognition (e.g., OpenAI Whisper or similar)
        # 2. Natural language processing to extract intent
        # 3. Contextual understanding for humanoid-specific commands
        
        # For simulation, we'll return a simulated command
        # In practice, this would interface with Isaac ROS audio processing nodes
        return "move forward 1 meter"  # Simulated recognized command

def main(args=None):
    rclpy.init(args=args)
    audio_perceptor = IsaacROSAudioPerceptor()
    
    try:
        rclpy.spin(audio_perceptor)
    except KeyboardInterrupt:
        audio_perceptor.get_logger().info('Shutting down audio perceptor...')
    finally:
        audio_perceptor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multimodal Perception Fusion

Combining multiple sensory inputs improves perception robustness:

```python
# multimodal_fusion.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

class IsaacROSMultimodalFusion(Node):
    def __init__(self):
        super().__init__('isaac_ros_multimodal_fusion')
        
        # Create subscribers for different modalities
        self.visual_sub = Subscriber(self, Image, '/camera/image_rect_color')
        self.depth_sub = Subscriber(self, PointCloud2, '/camera/depth')
        self.audio_sub = Subscriber(self, String, '/recognized_commands')
        
        # Synchronize messages from different modalities
        self.ts = ApproximateTimeSynchronizer(
            [self.visual_sub, self.depth_sub, self.audio_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.multimodal_callback)
        
        # Publish fused perception results
        self.fusion_publisher = self.create_publisher(
            String,  # In practice, this would be a more complex message type
            '/fused_perception',
            10)
        
        self.get_logger().info('Isaac ROS Multimodal Fusion initialized')

    def multimodal_callback(self, visual_msg, depth_msg, audio_msg):
        """Fuse information from multiple sensory modalities"""
        # Process visual information
        visual_analysis = self.analyze_visual_data(visual_msg)
        
        # Process depth information
        depth_analysis = self.analyze_depth_data(depth_msg)
        
        # Process audio information
        audio_analysis = self.analyze_audio_data(audio_msg)
        
        # Fuse the information
        fused_result = self.fuse_modalities(visual_analysis, depth_analysis, audio_analysis)
        
        # Publish the fused result
        result_msg = String()
        result_msg.data = fused_result
        self.fusion_publisher.publish(result_msg)
        
        self.get_logger().info(f'Fused perception result: {fused_result}')

    def analyze_visual_data(self, visual_msg):
        """Analyze visual information"""
        # This would perform computer vision tasks
        # such as object detection, scene understanding, etc.
        return "visual_analysis_result"

    def analyze_depth_data(self, depth_msg):
        """Analyze depth information"""
        # This would perform 3D understanding tasks
        # such as obstacle detection, surface normal estimation, etc.
        return "depth_analysis_result"

    def analyze_audio_data(self, audio_msg):
        """Analyze audio information"""
        # This would interpret voice commands and environmental sounds
        return "audio_analysis_result"

    def fuse_modalities(self, visual_result, depth_result, audio_result):
        """Fuse information from different modalities"""
        # Implement fusion algorithm
        # This could be a simple combination or a more sophisticated
        # neural network-based fusion approach
        return f"Fused: {visual_result}, {depth_result}, {audio_result}"

def main(args=None):
    rclpy.init(args=args)
    fusion_node = IsaacROSMultimodalFusion()
    
    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Shutting down multimodal fusion...')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Isaac Sim Integration for Perception Training

Isaac Sim provides an excellent environment for generating training data for perception systems:

### Synthetic Data Generation for Perception Models

```python
# perception_training_data_generator.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
from pxr import Gf
import numpy as np
import cv2
import os

class PerceptionTrainingDataGenerator:
    def __init__(self, output_dir="./perception_training_data"):
        self.output_dir = output_dir
        self.world = World(stage_units_in_meters=1.0)
        self.cameras = []
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(f"{output_dir}/rgb", exist_ok=True)
        os.makedirs(f"{output_dir}/depth", exist_ok=True)
        os.makedirs(f"{output_dir}/seg", exist_ok=True)
        
    def setup_scene(self):
        """Set up a scene for synthetic data generation"""
        # Add ground plane
        add_reference_to_stage(usd_path="/Isaac/Props/Grid/default_unit_cube_prim.usda", prim_path="/World/GroundPlane")
        
        # Add objects for detection
        self.add_objects_to_scene()
        
        # Add cameras at different positions
        self.add_cameras()
        
    def add_objects_to_scene(self):
        """Add objects to the scene for training data"""
        # Add various objects with different shapes, colors, and materials
        # This would be expanded in a real implementation
        pass
        
    def add_cameras(self):
        """Add cameras for multi-view training data"""
        # Add front-facing camera
        front_camera = Camera(
            prim_path="/World/FrontCamera",
            position=np.array([0.0, 0.0, 1.0]),
            frequency=30,
            resolution=(640, 480)
        )
        self.cameras.append(front_camera)
        
        # Add left-side camera
        left_camera = Camera(
            prim_path="/World/LeftCamera",
            position=np.array([-0.5, 0.5, 1.0]),
            frequency=30,
            resolution=(640, 480)
        )
        self.cameras.append(left_camera)
        
        # Add top-down camera
        top_camera = Camera(
            prim_path="/World/TopCamera",
            position=np.array([0.0, 0.0, 2.0]),
            frequency=30,
            resolution=(640, 480)
        )
        self.cameras.append(top_camera)
    
    def generate_training_data(self, num_samples=1000):
        """Generate synthetic training data samples"""
        for i in range(num_samples):
            # Randomize scene configuration
            self.randomize_scene()
            
            # Step the simulation to update scene
            self.world.step(render=True)
            
            # Capture data from all cameras
            for j, camera in enumerate(self.cameras):
                rgb = camera.get_rgb()
                depth = camera.get_depth()
                seg = camera.get_semantic_segmentation()
                
                # Save data with appropriate naming
                cv2.imwrite(f"{self.output_dir}/rgb/sample_{i:06d}_cam_{j}.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                np.save(f"{self.output_dir}/depth/sample_{i:06d}_cam_{j}.npy", depth)
                cv2.imwrite(f"{self.output_dir}/seg/sample_{i:06d}_cam_{j}.png", seg)
            
            print(f"Generated sample {i+1}/{num_samples}")
    
    def randomize_scene(self):
        """Randomize the scene for domain randomization"""
        # Randomize object positions, orientations, and appearances
        # Randomize lighting conditions
        # Randomize camera parameters
        pass

# Example usage
# generator = PerceptionTrainingDataGenerator()
# generator.setup_scene()
# generator.generate_training_data(num_samples=1000)
```

## Performance Optimization for Perception Systems

### Efficient Pipeline Design

```python
# optimized_perception_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import torch
import numpy as np
from threading import Thread, Lock
from queue import Queue

class OptimizedPerceptionPipeline(Node):
    def __init__(self):
        super().__init__('optimized_perception_pipeline')
        
        # Use threading for improved performance
        self.input_queue = Queue(maxsize=5)  # Limit queue size to manage memory
        self.output_queue = Queue(maxsize=5)
        self.processing_thread = None
        self.thread_running = False
        self.thread_lock = Lock()
        
        # Subscribe to camera input (using compressed image for efficiency)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.compressed_image_callback,
            10)
        
        # Publish detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/optimized_detections',
            10)
        
        self.bridge = CvBridge()
        
        # Initialize model once and reuse
        self.initialize_model()
        
        # Start processing thread
        self.start_processing_thread()
        
        self.get_logger().info('Optimized Perception Pipeline initialized')

    def initialize_model(self):
        """Initialize model once for reuse"""
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()
        
        # Move to GPU if available
        if torch.cuda.is_available():
            self.model.cuda()
            self.get_logger().info('Model loaded on GPU')
        
        # Warm up model to avoid first-run latency
        dummy_input = torch.zeros(1, 3, 480, 640)
        if torch.cuda.is_available():
            dummy_input = dummy_input.cuda()
        
        with torch.no_grad():
            _ = self.model(dummy_input)
        
        self.get_logger().info('Model warmed up')

    def start_processing_thread(self):
        """Start the processing thread"""
        self.thread_running = True
        self.processing_thread = Thread(target=self.processing_loop)
        self.processing_thread.start()

    def compressed_image_callback(self, msg):
        """Handle compressed image input"""
        try:
            # Only add to queue if there's space
            if not self.input_queue.full():
                with self.thread_lock:
                    self.input_queue.put(msg)
            else:
                self.get_logger().warn('Input queue full, dropping frame')
        except Exception as e:
            self.get_logger().error(f'Error adding to queue: {e}')

    def processing_loop(self):
        """Processing loop running in separate thread"""
        while self.thread_running:
            try:
                # Get image from queue (with timeout to allow checking thread_running)
                msg = self.input_queue.get(timeout=1.0)
                
                # Process image
                detections = self.process_image(msg)
                
                # Publish results
                if not self.output_queue.full():
                    self.output_queue.put((msg.header, detections))
                    # Publish from main thread via timer
                    self.timer = self.create_timer(0.01, self.publish_results)
                else:
                    self.get_logger().warn('Output queue full, dropping results')
                    
            except Exception as e:
                if "timeout" not in str(e):  # Don't log timeout errors
                    self.get_logger().error(f'Error in processing loop: {e}')

    def process_image(self, compressed_msg):
        """Process compressed image and return detections"""
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(compressed_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().error('Could not decode compressed image')
                return None
            
            # Run inference
            results = self.model(cv_image)
            
            # Convert results to ROS format
            detections = Detection2DArray()
            detections.header = compressed_msg.header
            
            for *xyxy, conf, cls in results.xyxy[0].tolist():
                if conf > 0.5:  # Confidence threshold
                    detection = Detection2D()
                    detection.header = compressed_msg.header
                    
                    # Convert bounding box coordinates
                    x_center = (xyxy[0] + xyxy[2]) / 2.0
                    y_center = (xyxy[1] + xyxy[3]) / 2.0
                    width = xyxy[2] - xyxy[0]
                    height = xyxy[3] - xyxy[1]
                    
                    detection.bbox.center.x = x_center
                    detection.bbox.center.y = y_center
                    detection.bbox.size_x = width
                    detection.bbox.size_y = height
                    
                    # Add hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = int(cls)
                    hypothesis.score = conf
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
        except Exception as e:
            # Queue might be empty, which is normal
            pass

    def destroy_node(self):
        """Clean up resources"""
        self.thread_running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    perception_pipeline = OptimizedPerceptionPipeline()
    
    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        perception_pipeline.get_logger().info('Shutting down optimized perception pipeline...')
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Validation and Testing

### Perception System Validation

```python
# perception_validator.py
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
import numpy as np

class PerceptionValidator(Node):
    def __init__(self):
        super().__init__('perception_validator')
        
        # Subscribe to detection results
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10)
        
        # Publish validation metrics
        self.accuracy_publisher = self.create_publisher(
            Float32,
            '/perception_accuracy',
            10)
        
        self.detection_latency_publisher = self.create_publisher(
            Float32,
            '/perception_latency',
            10)
        
        # Track metrics
        self.detection_times = []
        self.confidence_scores = []
        
        self.get_logger().info('Perception Validator initialized')

    def detection_callback(self, msg):
        """Validate perception results"""
        # Calculate detection latency
        current_time = self.get_clock().now().nanoseconds / 1e9
        detection_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        latency = current_time - detection_time
        self.detection_times.append(latency)
        
        # Calculate average confidence
        if msg.detections:
            confidences = [d.results[0].score if d.results else 0.0 for d in msg.detections]
            avg_confidence = sum(confidences) / len(confidences)
            self.confidence_scores.append(avg_confidence)
        
        # Publish metrics
        if len(self.detection_times) > 10:  # Publish every 10 detections
            avg_latency = sum(self.detection_times[-10:]) / min(10, len(self.detection_times))
            avg_confidence = sum(self.confidence_scores[-10:]) / min(10, len(self.confidence_scores))
            
            latency_msg = Float32()
            latency_msg.data = avg_latency
            self.detection_latency_publisher.publish(latency_msg)
            
            accuracy_msg = Float32()
            accuracy_msg.data = avg_confidence
            self.accuracy_publisher.publish(accuracy_msg)
            
            self.get_logger().info(f'Avg latency: {avg_latency:.3f}s, Avg confidence: {avg_confidence:.3f}')

def main(args=None):
    rclpy.init(args=args)
    validator = PerceptionValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Shutting down perception validator...')
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for AI Perception in Humanoid Robotics

### 1. Robustness Considerations
- Design perception systems to handle challenging lighting conditions
- Implement fallback mechanisms when primary sensors fail
- Account for humanoid-specific challenges (self-occlusion, dynamic poses)

### 2. Efficiency Considerations
- Optimize inference for real-time performance
- Use appropriate model sizes for computational constraints
- Implement selective processing based on task requirements

### 3. Safety Considerations
- Validate perception accuracy before taking actions
- Implement confidence thresholds for safety-critical operations
- Design perception systems to detect and handle anomalies

## Summary

AI perception systems are fundamental to humanoid robotics, enabling robots to understand and interact with their environment. This section covered visual perception, audio perception, multimodal fusion, and optimization techniques for perception systems in the context of Isaac Sim and Isaac ROS.

In the next section, we'll put these concepts into practice with hands-on exercises that demonstrate implementing perception systems for humanoid robotics applications.