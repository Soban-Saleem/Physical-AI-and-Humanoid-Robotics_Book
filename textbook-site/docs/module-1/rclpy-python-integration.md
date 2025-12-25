# Python Integration with rclpy

Python is one of the most popular languages for robotics development, particularly for AI and machine learning applications. In this section, we'll explore how to integrate Python with ROS 2 using rclpy, the Python client library for ROS 2. This integration is essential for creating AI-powered robotic systems that can process sensor data, make decisions, and control robot behavior.

## Learning Objectives

By the end of this section, you will be able to:
- Use rclpy to create ROS 2 nodes in Python
- Integrate Python AI libraries with ROS 2 nodes
- Process sensor data from ROS 2 topics in Python
- Create services and actions using Python
- Implement parameter handling in Python nodes
- Apply best practices for Python-ROS 2 integration
- Design efficient data processing pipelines using Python

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API for creating nodes, publishers, subscribers, services, and actions. It's built on top of the ROS Client Library (rcl) and the underlying DDS implementation.

### Why Python for Robotics?

Python is particularly well-suited for robotics because:
- Rich ecosystem of scientific computing libraries (NumPy, SciPy, Pandas)
- Extensive machine learning frameworks (TensorFlow, PyTorch, scikit-learn)
- Rapid prototyping capabilities
- Strong community support for robotics and AI
- Integration with computer vision libraries (OpenCV, PIL)
- Natural language processing capabilities

### Installing and Importing rclpy

rclpy is part of the standard ROS 2 installation and can be imported as follows:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
```

## Creating Nodes with rclpy

Let's start by creating a basic ROS 2 node in Python:

```python
# File: humanoid_ws/src/humanoid_ai/humanoid_ai/basic_node_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from builtin_interfaces.msg import Time


class BasicPythonNode(Node):

    def __init__(self):
        super().__init__('basic_python_node')
        
        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.listener_callback,
            10)
        
        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Counter for messages
        self.i = 0
        
        self.get_logger().info('Basic Python Node initialized')

    def timer_callback(self):
        """Publish a message every timer tick"""
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        """Handle incoming messages"""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    basic_node = BasicPythonNode()

    try:
        rclpy.spin(basic_node)
    except KeyboardInterrupt:
        basic_node.get_logger().info('Keyboard interrupt caught, shutting down...')
    finally:
        basic_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Integrating AI Libraries with ROS 2

One of the key advantages of using Python with ROS 2 is the ability to integrate AI libraries seamlessly. Let's create a node that processes sensor data using machine learning:

```python
# File: humanoid_ws/src/humanoid_ai/humanoid_ai/ai_perception_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn.cluster import KMeans


class AI Perception Node(Node):

    def __init__(self):
        super().__init__('ai_perception_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        
        # Publishers
        self.perception_publisher = self.create_publisher(
            String,
            'perception_output',
            10)
        
        # Internal state
        self.latest_image = None
        self.object_detector = None  # Will initialize later
        
        # Timer for processing
        self.processing_timer = self.create_timer(0.1, self.process_data)
        
        self.get_logger().info('AI Perception Node initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Store the latest image for processing
            self.latest_image = cv_image
            
            # In a real implementation, you might run object detection here
            # self.run_object_detection(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        # Extract ranges from LiDAR scan
        ranges = np.array(msg.ranges)
        
        # Filter out invalid ranges (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        # In a real implementation, you might detect obstacles here
        if len(valid_ranges) > 0:
            min_range = np.min(valid_ranges)
            if min_range < 0.5:  # Less than 50 cm away
                self.get_logger().warn('Obstacle detected nearby!')

    def process_data(self):
        """Process sensor data using AI algorithms"""
        if self.latest_image is not None:
            # Process the latest image
            processed_result = self.process_image_with_ai(self.latest_image)
            
            # Publish results
            if processed_result:
                result_msg = String()
                result_msg.data = processed_result
                self.perception_publisher.publish(result_msg)

    def process_image_with_ai(self, image):
        """Apply AI algorithms to process the image"""
        try:
            # Example: Simple color clustering using K-means
            # Reshape image for clustering
            pixels = image.reshape((-1, 3))
            pixels = np.float32(pixels)
            
            # Apply K-means clustering
            kmeans = KMeans(n_clusters=3, random_state=42)
            labels = kmeans.fit_predict(pixels)
            
            # Get dominant colors
            centers = np.uint8(kmeans.cluster_centers_)
            
            # For this example, just return information about the dominant colors
            result_str = f"Dominant colors detected: {centers.tolist()}"
            
            return result_str
        except Exception as e:
            self.get_logger().error(f'Error in AI processing: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    ai_perception_node = AI Perception Node()
    
    try:
        rclpy.spin(ai_perception_node)
    except KeyboardInterrupt:
        ai_perception_node.get_logger().info('Shutting down AI Perception Node...')
    finally:
        ai_perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Working with Parameters in Python

Parameters allow you to configure your nodes at runtime. Here's how to work with parameters in rclpy:

```python
# File: humanoid_ws/src/humanoid_ai/humanoid_ai/parameter_node.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('max_objects', 10)
        self.declare_parameter('model_path', '/default/model/path')
        self.declare_parameter('use_gpu', True)
        
        # Create publisher
        self.param_status_pub = self.create_publisher(String, 'parameter_status', 10)
        
        # Timer to periodically publish parameter status
        self.status_timer = self.create_timer(5.0, self.publish_param_status)
        
        # Callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Parameter Node initialized')

    def parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        
        # Return success for all parameters
        return SetParametersResult(successful=True)

    def publish_param_status(self):
        """Publish current parameter status"""
        # Get parameter values
        threshold = self.get_parameter('detection_threshold').value
        max_objects = self.get_parameter('max_objects').value
        model_path = self.get_parameter('model_path').value
        use_gpu = self.get_parameter('use_gpu').value
        
        status_msg = String()
        status_msg.data = (
            f"Threshold: {threshold}, Max Objects: {max_objects}, "
            f"Model: {model_path}, GPU: {use_gpu}"
        )
        
        self.param_status_pub.publish(status_msg)
        
        self.get_logger().info(f'Published parameter status: {status_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    
    param_node = ParameterNode()
    
    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        param_node.get_logger().info('Shutting down parameter node...')
    finally:
        param_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced: Using AI Models with ROS 2

Now let's create a more advanced example that uses a deep learning model for perception:

```python
# File: humanoid_ws/src/humanoid_ai/humanoid_ai/deep_learning_perception.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from humanoid_msgs.msg import ObjectDetectionArray, ObjectDetection
from cv_bridge import CvBridge
import torch
import torchvision.transforms as T
from torchvision.models import mobilenet_v2


class DeepLearningPerception(Node):

    def __init__(self):
        super().__init__('deep_learning_perception')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize model (using MobileNetV2 for efficiency on robot)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = mobilenet_v2(pretrained=True)
        self.model.eval()
        self.model.to(self.device)
        
        # Image preprocessing
        self.transform = T.Compose([
            T.ToPILImage(),
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], 
                       std=[0.229, 0.224, 0.225])
        ])
        
        # Class labels (simplified - normally you'd load from file)
        self.imagenet_classes = self.load_imagenet_labels()
        
        # Subscribers and publishers
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.detection_publisher = self.create_publisher(
            ObjectDetectionArray,
            'object_detections',
            10)
        
        self.get_logger().info('Deep Learning Perception Node initialized')

    def load_imagenet_labels(self):
        """Load ImageNet class labels"""
        # In a real implementation, you would load these from a file
        # For this example, we'll just return a simplified version
        return {
            0: 'tench', 1: 'goldfish', 2: 'great_white_shark', 
            3: 'tiger_shark', 4: 'hammerhead', 5: 'electric_ray',
            # ... more classes would be defined
            207: 'orange', 208: 'lemon', 212: 'fig', 213: 'pineapple',
            # ... more classes
            393: 'lakeland_terrier', 394: 'norwich_terrier', 
            395: 'yorkshire_terrier', 396: 'wire-haired_fox_terrier',
            # ... more classes
            999: 'menu'
        }

    def image_callback(self, msg):
        """Process incoming image with deep learning model"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Preprocess the image
            input_tensor = self.transform(rgb_image).unsqueeze(0)
            input_tensor = input_tensor.to(self.device)
            
            # Run inference
            with torch.no_grad():
                outputs = self.model(input_tensor)
                probabilities = torch.nn.functional.softmax(outputs[0], dim=0)
                
                # Get top predictions
                top_probs, top_indices = torch.topk(probabilities, k=5)
                
                # Create detection array message
                detection_array = ObjectDetectionArray()
                detection_array.header = msg.header
                
                for prob, idx in zip(top_probs, top_indices):
                    confidence = prob.item()
                    
                    # Only include detections above threshold
                    if confidence > 0.1:  # 10% confidence threshold
                        detection = ObjectDetection()
                        detection.label = self.imagenet_classes.get(idx.item(), f'class_{idx.item()}')
                        detection.confidence = confidence
                        detection.class_id = idx.item()
                        
                        # For now, we'll just set dummy bounding box coordinates
                        # In a real implementation, you'd use an object detection model
                        detection.bounding_box.x_offset = 0
                        detection.bounding_box.y_offset = 0
                        detection.bounding_box.width = 100
                        detection.bounding_box.height = 100
                        
                        detection_array.detections.append(detection)
                
                # Publish detections
                self.detection_publisher.publish(detection_array)
                
                self.get_logger().info(
                    f'Detected objects: {[d.label for d in detection_array.detections]}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in deep learning processing: {str(e)}')

    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('Shutting down Deep Learning Perception Node...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    dl_perception_node = DeepLearningPerception()
    
    try:
        rclpy.spin(dl_perception_node)
    except KeyboardInterrupt:
        dl_perception_node.get_logger().info('Shutting down deep learning perception node...')
    finally:
        dl_perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Error Handling and Logging Best Practices

Proper error handling and logging are essential for robust ROS 2 nodes:

```python
# File: humanoid_ws/src/humanoid_ai/humanoid_ai/error_handling_example.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import traceback
import time


class RobustPythonNode(Node):

    def __init__(self):
        super().__init__('robust_python_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        # Publishers
        self.status_publisher = self.create_publisher(String, 'node_status', 10)
        
        # Statistics
        self.processed_images = 0
        self.error_count = 0
        
        # Timer for publishing status
        self.status_timer = self.create_timer(10.0, self.publish_status)
        
        self.get_logger().info('Robust Python Node initialized')

    def image_callback(self, msg):
        """Process incoming image with robust error handling"""
        try:
            # Basic validation
            if msg.width <= 0 or msg.height <= 0:
                self.get_logger().warn('Received image with invalid dimensions')
                return
            
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image (this would be your actual processing)
            result = self.process_image(cv_image)
            
            if result:
                # Update statistics
                self.processed_images += 1
                
                # Log performance if processing took too long
                if result.get('processing_time', 0) > 0.1:  # 100ms threshold
                    self.get_logger().warn(
                        f'Image processing took {result["processing_time"]:.3f}s, '
                        f'which exceeds recommended threshold'
                    )
            
        except cv2.error as e:
            self.get_logger().error(f'OpenCV error processing image: {str(e)}')
            self.error_count += 1
        except Exception as e:
            self.get_logger().error(f'Unexpected error processing image: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            self.error_count += 1

    def process_image(self, image):
        """Process image with performance monitoring"""
        start_time = time.time()
        
        try:
            # Simulate image processing
            # In a real implementation, this would be your actual processing
            height, width = image.shape[:2]
            
            # Example: simple processing
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Calculate processing time
            processing_time = time.time() - start_time
            
            return {
                'processed': True,
                'dimensions': (width, height),
                'processing_time': processing_time
            }
            
        except Exception as e:
            processing_time = time.time() - start_time
            self.get_logger().error(f'Error in image processing: {str(e)}')
            self.error_count += 1
            return None

    def publish_status(self):
        """Publish node status information"""
        status_msg = String()
        status_msg.data = (
            f"Processed: {self.processed_images}, "
            f"Errors: {self.error_count}, "
            f"Error Rate: {(self.error_count/(self.processed_images+self.error_count or 1))*100:.2f}%"
        )
        
        self.status_publisher.publish(status_msg)
        
        self.get_logger().info(f'Node status: {status_msg.data}')

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info(
            f'Node destroyed. Processed {self.processed_images} images, '
            f'encountered {self.error_count} errors.'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    robust_node = RobustPythonNode()
    
    try:
        rclpy.spin(robust_node)
    except KeyboardInterrupt:
        robust_node.get_logger().info('Shutting down robust node...')
    finally:
        robust_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Performance Optimization

For AI applications in robotics, performance is critical. Here are some optimization techniques:

### 1. Threading for CPU-intensive tasks

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import queue
from time import sleep


class ThreadingExampleNode(Node):

    def __init__(self):
        super().__init__('threading_example_node')
        
        self.publisher_ = self.create_publisher(String, 'processed_data', 10)
        
        # Queue for processing tasks
        self.processing_queue = queue.Queue()
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self.processing_worker)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        
        # Timer to add tasks to queue
        self.timer = self.create_timer(0.1, self.add_task_to_queue)
        
        self.task_counter = 0

    def add_task_to_queue(self):
        """Add a task to the processing queue"""
        task_data = f"Task {self.task_counter}"
        self.processing_queue.put(task_data)
        self.task_counter += 1

    def processing_worker(self):
        """Worker thread that processes tasks"""
        while rclpy.ok():
            try:
                # Get task from queue (with timeout to allow checking for shutdown)
                task_data = self.processing_queue.get(timeout=1.0)
                
                # Simulate CPU-intensive processing
                sleep(0.5)  # Replace with actual processing
                
                # Publish result
                result_msg = String()
                result_msg.data = f"Processed: {task_data}"
                
                # Use call_later to publish from main thread
                self.call_later(0, lambda: self.publisher_.publish(result_msg))
                
                self.processing_queue.task_done()
                
            except queue.Empty:
                # Queue was empty, continue loop
                continue
            except Exception as e:
                self.get_logger().error(f'Error in processing worker: {str(e)}')
                continue

    def destroy_node(self):
        """Clean up before destroying node"""
        if self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()
```

### 2. Memory Management

```python
import rclpy
from rclpy.node import Node
import gc
import psutil
import os


class MemoryEfficientNode(Node):

    def __init__(self):
        super().__init__('memory_efficient_node')
        
        # Monitor memory usage
        self.memory_timer = self.create_timer(5.0, self.check_memory_usage)
        
        # For processing large data, use generators and iterators
        # rather than loading everything into memory at once
        self.large_data_processor = LargeDataProcessor()
        
        self.get_logger().info('Memory efficient node initialized')

    def check_memory_usage(self):
        """Monitor and log memory usage"""
        process = psutil.Process(os.getpid())
        memory_mb = process.memory_info().rss / 1024 / 1024
        
        self.get_logger().info(f'Memory usage: {memory_mb:.2f} MB')
        
        # If memory usage is high, trigger garbage collection
        if memory_mb > 500:  # 500 MB threshold
            self.get_logger().warn('High memory usage detected, running GC')
            gc.collect()
            self.get_logger().info('Garbage collection completed')
```

## Summary

In this section, we've explored how to integrate Python with ROS 2 using rclpy. We've covered:

1. Creating basic ROS 2 nodes in Python
2. Integrating AI libraries with ROS 2 nodes
3. Working with parameters for configuration
4. Implementing advanced perception using deep learning models
5. Applying error handling and logging best practices
6. Optimizing performance for AI applications

The integration of Python with ROS 2 is particularly powerful for humanoid robotics because it allows us to leverage the rich ecosystem of AI and machine learning libraries while maintaining the distributed architecture benefits of ROS 2.

In the next section, we'll explore how to describe robots using URDF (Unified Robot Description Format), which is essential for simulation and visualization in robotics applications.