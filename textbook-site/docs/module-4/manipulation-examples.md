# Manipulation with Humanoid Hands

Manipulation is a core capability for humanoid robots, enabling them to interact with objects in their environment. This section covers how to implement manipulation systems that leverage computer vision, AI planning, and precise control to enable humanoid robots to grasp, move, and manipulate objects effectively.

## Learning Objectives

By the end of this section, you will be able to:
- Understand the kinematics and dynamics of humanoid manipulation
- Implement grasp planning algorithms for humanoid hands
- Integrate vision-based object detection with manipulation planning
- Create manipulation action sequences for complex tasks
- Implement safety checks and validation for manipulation actions
- Connect manipulation planning to ROS 2 control interfaces
- Optimize manipulation performance for real-world scenarios

## Introduction to Humanoid Manipulation

Humanoid manipulation differs significantly from traditional robotic manipulation due to the anthropomorphic nature of the hands and the need to consider the robot's whole-body dynamics. Key challenges include:

- **Anthropomorphic Hand Design**: Humanoid hands have complex kinematics with multiple DOF per finger
- **Whole-Body Coordination**: Manipulation requires coordination of arms, torso, and potentially legs for balance
- **Grasp Stability**: Ensuring stable grasps with potentially underactuated humanoid hands
- **Force Control**: Managing forces during contact with objects
- **Collision Avoidance**: Avoiding self-collisions and environmental collisions during manipulation

## Humanoid Hand Kinematics

Humanoid robots typically have hands with 4-5 fingers and 3-4 joints per finger, resulting in 12-20 degrees of freedom per hand. This allows for dexterous manipulation but requires sophisticated control algorithms.

### Hand Model Structure

A typical humanoid hand might have the following structure:

```
Hand (base)
├── Thumb
│   ├── Thumb proximal joint (abduction/adduction)
│   ├── Thumb intermediate joint (flexion/extension)
│   └── Thumb distal joint (flexion/extension)
├── Index Finger
│   ├── Index proximal joint (flexion/extension)
│   ├── Index intermediate joint (flexion/extension)
│   └── Index distal joint (flexion/extension)
├── Middle Finger
│   ├── Middle proximal joint (flexion/extension)
│   ├── Middle intermediate joint (flexion/extension)
│   └── Middle distal joint (flexion/extension)
├── Ring Finger
│   ├── Ring proximal joint (flexion/extension)
│   ├── Ring intermediate joint (flexion/extension)
│   └── Ring distal joint (flexion/extension)
└── Pinky Finger
    ├── Pinky proximal joint (flexion/extension)
    ├── Pinky intermediate joint (flexion/extension)
    └── Pinky distal joint (flexion/extension)
```

## Grasp Planning

Grasp planning involves determining how to position the hand to grasp an object stably. For humanoid robots, this involves:

1. **Object Analysis**: Understanding object shape, size, and material properties
2. **Grasp Candidate Generation**: Generating potential grasp configurations
3. **Grasp Evaluation**: Evaluating the stability and feasibility of grasps
4. **Grasp Execution**: Moving the hand to execute the chosen grasp

### Grasp Planning Node Implementation

```python
# grasp_planner.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R

class IsaacROSGraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')
        
        # Subscriptions
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10)
        
        self.object_pose_subscription = self.create_subscription(
            Pose,
            '/object_pose',
            self.object_pose_callback,
            10)
        
        # Publishers
        self.grasp_plan_publisher = self.create_publisher(
            String,  # In practice, use a structured grasp plan message
            '/grasp_plans',
            10)
        
        self.joint_command_publisher = self.create_publisher(
            JointState,
            '/joint_group_position_controller/commands',
            10)
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize hand kinematics model
        self.hand_model = self.initialize_hand_model()
        
        # Store object information
        self.objects = {}
        self.selected_object = None
        
        self.get_logger().info('Isaac ROS Grasp Planner initialized')

    def initialize_hand_model(self):
        """Initialize the hand kinematics model"""
        # In a real implementation, this would load a URDF or use a kinematics library
        # For this example, we'll define basic hand properties
        hand_model = {
            'fingers': ['thumb', 'index', 'middle', 'ring', 'pinky'],
            'joints_per_finger': 3,
            'max_grasp_force': 50.0,  # Newtons
            'finger_lengths': [0.03, 0.07, 0.04],  # Proximal, intermediate, distal segments
            'hand_width': 0.1,  # meters
            'hand_height': 0.05,  # meters
        }
        return hand_model

    def detection_callback(self, msg):
        """Process object detections for potential manipulation"""
        for detection in msg.detections:
            if detection.results and len(detection.results) > 0:
                # Get the best result (highest confidence)
                best_result = max(detection.results, key=lambda x: x.score)
                
                # Store object information
                self.objects[best_result.id] = {
                    'bbox': detection.bbox,
                    'confidence': best_result.score,
                    'timestamp': self.get_clock().now().nanoseconds
                }
        
        self.get_logger().info(f'Updated object knowledge: {list(self.objects.keys())}')

    def object_pose_callback(self, msg):
        """Receive object pose for manipulation planning"""
        # Plan grasp based on object pose
        grasp_plan = self.plan_grasp_for_object(msg)
        
        if grasp_plan:
            # Publish grasp plan
            grasp_msg = String()
            grasp_msg.data = str(grasp_plan)
            self.grasp_plan_publisher.publish(grasp_msg)
            
            self.get_logger().info(f'Published grasp plan for object at {msg.position}')

    def plan_grasp_for_object(self, object_pose):
        """Plan a grasp for the specified object pose"""
        try:
            # Calculate approach position (slightly above the object)
            approach_pose = Pose()
            approach_pose.position.x = object_pose.position.x
            approach_pose.position.y = object_pose.position.y
            approach_pose.position.z = object_pose.position.z + 0.1  # 10cm above object
            
            # Orient hand to approach from above (typical for top grasp)
            approach_rotation = R.from_euler('xyz', [0, 1.57, 0])  # Rotate 90 degrees around Y
            quat = approach_rotation.as_quat()
            approach_pose.orientation.x = quat[0]
            approach_pose.orientation.y = quat[1]
            approach_pose.orientation.z = quat[2]
            approach_pose.orientation.w = quat[3]
            
            # Calculate grasp position (at object level)
            grasp_pose = Pose()
            grasp_pose.position.x = object_pose.position.x
            grasp_pose.position.y = object_pose.position.y
            grasp_pose.position.z = object_pose.position.z + 0.05  # 5cm above object base
            
            # Copy orientation from approach
            grasp_pose.orientation = approach_pose.orientation
            
            # Calculate lift position (after grasping)
            lift_pose = Pose()
            lift_pose.position.x = object_pose.position.x
            lift_pose.position.y = object_pose.position.y
            lift_pose.position.z = object_pose.position.z + 0.2  # Lift 20cm
            lift_pose.orientation = approach_pose.orientation
            
            # Generate grasp plan
            grasp_plan = {
                'object_pose': object_pose,
                'approach_pose': approach_pose,
                'grasp_pose': grasp_pose,
                'lift_pose': lift_pose,
                'grasp_type': 'top_grasp',
                'finger_positions': self.calculate_finger_positions(object_pose),
                'grasp_force': 10.0  # Moderate grasp force
            }
            
            return grasp_plan
            
        except Exception as e:
            self.get_logger().error(f'Error planning grasp: {e}')
            return None

    def calculate_finger_positions(self, object_pose):
        """Calculate appropriate finger positions for grasping the object"""
        # This is a simplified calculation - in reality, this would involve
        # complex inverse kinematics and grasp stability analysis
        
        # For a cylindrical object, we might use a tripod grasp:
        # thumb opposing index and middle fingers
        finger_positions = {
            'thumb': [0.2, 0.0, 0.0],  # Opposing position
            'index': [-0.1, 0.05, 0.0],   # Side grip
            'middle': [-0.1, -0.05, 0.0],  # Side grip
            'ring': [0.0, 0.0, 0.0],   # Support
            'pinky': [0.0, 0.0, 0.0]   # Support
        }
        
        return finger_positions

def main(args=None):
    rclpy.init(args=args)
    grasp_planner = IsaacROSGraspPlanner()
    
    try:
        rclpy.spin(grasp_planner)
    except KeyboardInterrupt:
        grasp_planner.get_logger().info('Shutting down grasp planner...')
    finally:
        grasp_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Manipulation Action Sequences

For complex manipulation tasks, we need to sequence multiple actions:

### Example: Pick and Place Action Sequence

```python
# manipulation_sequencer.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
import time

class ManipulationSequencer(Node):
    def __init__(self):
        super().__init__('manipulation_sequencer')
        
        # Subscriptions
        self.manipulation_command_subscription = self.create_subscription(
            String,
            '/manipulation_commands',
            self.command_callback,
            10)
        
        # Publishers
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/manipulation_trajectory',
            10)
        
        self.status_publisher = self.create_publisher(
            String,
            '/manipulation_status',
            10)
        
        # Store manipulation state
        self.current_manipulation_state = 'idle'
        self.manipulation_sequence = []
        self.current_step = 0
        
        self.get_logger().info('Manipulation Sequencer initialized')

    def command_callback(self, msg):
        """Process manipulation commands"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('command_type')
            
            if command_type == 'pick_and_place':
                self.execute_pick_and_place(command_data)
            elif command_type == 'transport_object':
                self.execute_transport(command_data)
            elif command_type == 'assemble_parts':
                self.execute_assembly(command_data)
            else:
                self.get_logger().warn(f'Unknown manipulation command: {command_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in manipulation command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing manipulation command: {e}')

    def execute_pick_and_place(self, command_data):
        """Execute pick and place sequence"""
        # Extract parameters
        object_pose = command_data.get('object_pose')
        target_pose = command_data.get('target_pose')
        object_name = command_data.get('object_name', 'unknown_object')
        
        # Define manipulation sequence
        sequence = [
            {
                'action': 'approach_object',
                'pose': self.calculate_approach_pose(object_pose),
                'description': f'Approach {object_name}'
            },
            {
                'action': 'descend_to_object',
                'pose': self.calculate_grasp_pose(object_pose),
                'description': f'Descend to grasp {object_name}'
            },
            {
                'action': 'grasp_object',
                'parameters': {'force': 15.0, 'object_name': object_name},
                'description': f'Grasp {object_name}'
            },
            {
                'action': 'lift_object',
                'pose': self.calculate_lift_pose(object_pose),
                'description': f'Lift {object_name}'
            },
            {
                'action': 'navigate_to_target',
                'pose': self.calculate_transport_pose(target_pose),
                'description': f'Transport {object_name} to target'
            },
            {
                'action': 'descend_to_place',
                'pose': self.calculate_place_pose(target_pose),
                'description': f'Descend to place {object_name}'
            },
            {
                'action': 'release_object',
                'parameters': {'object_name': object_name},
                'description': f'Release {object_name}'
            },
            {
                'action': 'retreat_from_object',
                'pose': self.calculate_retreat_pose(target_pose),
                'description': f'Retreat after placing {object_name}'
            }
        ]
        
        self.execute_manipulation_sequence(sequence)

    def calculate_approach_pose(self, object_pose):
        """Calculate approach pose above the object"""
        approach = Pose()
        approach.position.x = object_pose.position.x
        approach.position.y = object_pose.position.y
        approach.position.z = object_pose.position.z + 0.2  # 20cm above object
        approach.orientation = object_pose.orientation
        return approach

    def calculate_grasp_pose(self, object_pose):
        """Calculate grasp pose at the object"""
        grasp = Pose()
        grasp.position.x = object_pose.position.x
        grasp.position.y = object_pose.position.y
        grasp.position.z = object_pose.position.z + 0.05  # 5cm above base
        grasp.orientation = object_pose.orientation
        return grasp

    def calculate_lift_pose(self, object_pose):
        """Calculate lift pose after grasping"""
        lift = Pose()
        lift.position.x = object_pose.position.x
        lift.position.y = object_pose.position.y
        lift.position.z = object_pose.position.z + 0.3  # 30cm lift
        lift.orientation = object_pose.orientation
        return lift

    def calculate_transport_pose(self, target_pose):
        """Calculate transport pose to move object to target"""
        transport = Pose()
        transport.position.x = target_pose.position.x
        transport.position.y = target_pose.position.y
        transport.position.z = target_pose.position.z + 0.3  # Maintain height during transport
        transport.orientation = target_pose.orientation
        return transport

    def calculate_place_pose(self, target_pose):
        """Calculate place pose at target location"""
        place = Pose()
        place.position.x = target_pose.position.x
        place.position.y = target_pose.position.y
        place.position.z = target_pose.position.z + 0.05  # 5cm above target surface
        place.orientation = target_pose.orientation
        return place

    def calculate_retreat_pose(self, target_pose):
        """Calculate retreat pose after placing"""
        retreat = Pose()
        retreat.position.x = target_pose.position.x
        retreat.position.y = target_pose.position.y
        retreat.position.z = target_pose.position.z + 0.2  # 20cm above target
        retreat.orientation = target_pose.orientation
        return retreat

    def execute_manipulation_sequence(self, sequence):
        """Execute a sequence of manipulation actions"""
        self.manipulation_sequence = sequence
        self.current_step = 0
        self.current_manipulation_state = 'executing'
        
        self.get_logger().info(f'Starting manipulation sequence with {len(sequence)} steps')
        
        # Execute the first step
        self.execute_next_step()

    def execute_next_step(self):
        """Execute the next step in the manipulation sequence"""
        if self.current_step >= len(self.manipulation_sequence):
            # Sequence complete
            self.current_manipulation_state = 'complete'
            status_msg = String()
            status_msg.data = 'MANIPULATION_COMPLETE'
            self.status_publisher.publish(status_msg)
            self.get_logger().info('Manipulation sequence complete')
            return
        
        # Get current step
        step = self.manipulation_sequence[self.current_step]
        self.get_logger().info(f'Executing step {self.current_step + 1}: {step["description"]}')
        
        # Execute the action based on type
        if step['action'] == 'approach_object':
            self.execute_approach(step['pose'])
        elif step['action'] == 'descend_to_object':
            self.execute_descend(step['pose'])
        elif step['action'] == 'grasp_object':
            self.execute_grasp(step['parameters'])
        elif step['action'] == 'lift_object':
            self.execute_lift(step['pose'])
        elif step['action'] == 'navigate_to_target':
            self.execute_transport(step['pose'])
        elif step['action'] == 'descend_to_place':
            self.execute_descend(step['pose'])
        elif step['action'] == 'release_object':
            self.execute_release(step['parameters'])
        elif step['action'] == 'retreat_from_object':
            self.execute_retreat(step['pose'])
        
        # Update status
        status_msg = String()
        status_msg.data = f'STEP_{self.current_step + 1}_OF_{len(self.manipulation_sequence)}'
        self.status_publisher.publish(status_msg)
        
        # Schedule next step after delay (in a real implementation, this would be event-driven)
        self.current_step += 1
        self.create_timer(2.0, self.execute_next_step)  # 2-second delay between steps

    def execute_approach(self, pose):
        """Execute approach action"""
        # In a real implementation, this would send trajectory commands to move to the approach pose
        self.get_logger().info(f'Approaching pose: ({pose.position.x}, {pose.position.y}, {pose.position.z})')

    def execute_descend(self, pose):
        """Execute descent action"""
        self.get_logger().info(f'Descending to pose: ({pose.position.x}, {pose.position.y}, {pose.position.z})')

    def execute_grasp(self, params):
        """Execute grasp action"""
        self.get_logger().info(f'Grasping object: {params.get("object_name")} with force: {params.get("force")}N')

    def execute_lift(self, pose):
        """Execute lift action"""
        self.get_logger().info(f'Lifting to pose: ({pose.position.x}, {pose.position.y}, {pose.position.z})')

    def execute_transport(self, pose):
        """Execute transport action"""
        self.get_logger().info(f'Transporting to pose: ({pose.position.x}, {pose.position.y}, {pose.position.z})')

    def execute_release(self, params):
        """Execute release action"""
        self.get_logger().info(f'Releasing object: {params.get("object_name")}')

    def execute_retreat(self, pose):
        """Execute retreat action"""
        self.get_logger().info(f'Retreating to pose: ({pose.position.x}, {pose.position.y}, {pose.position.z})')

def main(args=None):
    rclpy.init(args=args)
    sequencer = ManipulationSequencer()
    
    try:
        rclpy.spin(sequencer)
    except KeyboardInterrupt:
        sequencer.get_logger().info('Shutting down manipulation sequencer...')
    finally:
        sequencer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Vision-Based Manipulation

Integrating computer vision with manipulation enables more robust and adaptive behavior:

### Example: Vision-Guided Grasping

```python
# vision_guided_grasping.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import tf2_ros

class VisionGuidedGrasping(Node):
    def __init__(self):
        super().__init__('vision_guided_grasping')
        
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
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)
        
        # Publishers
        self.manipulation_command_publisher = self.create_publisher(
            String,
            '/manipulation_commands',
            10)
        
        self.visualization_publisher = self.create_publisher(
            Image,
            '/grasp_visualization',
            10)
        
        # Initialize components
        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_matrix = None
        self.latest_detections = []
        
        self.get_logger().info('Vision-Guided Grasping Node initialized')

    def camera_info_callback(self, msg):
        """Store camera calibration information"""
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def detection_callback(self, msg):
        """Process object detections for grasping"""
        self.latest_detections = msg.detections
        
        # For each detection, calculate 3D position and plan grasp
        for detection in self.latest_detections:
            if detection.results and len(detection.results) > 0:
                # Get the best result
                best_result = max(detection.results, key=lambda x: x.score)
                
                if best_result.score > 0.7:  # Confidence threshold
                    # Calculate 3D position from 2D detection and depth
                    # This would require depth information in a real implementation
                    object_3d_pose = self.calculate_3d_pose_from_detection(
                        detection.bbox.center, 
                        best_result.id
                    )
                    
                    if object_3d_pose:
                        # Plan and execute grasp
                        self.plan_and_execute_grasp(object_3d_pose, best_result.id)

    def calculate_3d_pose_from_detection(self, bbox_center, object_id):
        """Calculate 3D pose from 2D detection (simplified - requires depth in real implementation)"""
        if not self.camera_matrix:
            return None
            
        # This is a simplified approach - in reality, you'd need depth information
        # or multiple views to reconstruct 3D position
        
        # For this example, we'll assume the object is at a known depth
        # and calculate the 3D position using camera intrinsics
        u = bbox_center.x
        v = bbox_center.y
        z = 1.0  # Known depth (in a real system, this would come from depth sensor)
        
        # Convert from pixel coordinates to 3D world coordinates
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # Create 3D pose (relative to camera frame)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Set orientation to face the object
        pose.orientation.w = 1.0  # Identity quaternion (facing forward)
        
        return pose

    def plan_and_execute_grasp(self, object_pose, object_id):
        """Plan and execute a grasp for the detected object"""
        self.get_logger().info(f'Planning grasp for {object_id} at 3D position: ({object_pose.position.x}, {object_pose.position.y}, {object_pose.position.z})')
        
        # In a real implementation, this would:
        # 1. Transform object pose from camera frame to robot base frame
        # 2. Plan an appropriate grasp based on object properties
        # 3. Execute the grasp sequence
        
        # For now, publish a manipulation command
        grasp_command = {
            'command_type': 'pick_and_place',
            'object_pose': {
                'position': {
                    'x': object_pose.position.x,
                    'y': object_pose.position.y,
                    'z': object_pose.position.z
                },
                'orientation': {
                    'x': object_pose.orientation.x,
                    'y': object_pose.orientation.y,
                    'z': object_pose.orientation.z,
                    'w': object_pose.orientation.w
                }
            },
            'target_pose': {
                'position': {
                    'x': object_pose.position.x + 0.5,  # Move 0.5m in x direction
                    'y': object_pose.position.y,
                    'z': object_pose.position.z
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0
                }
            },
            'object_name': object_id
        }
        
        command_msg = String()
        command_msg.data = json.dumps(grasp_command)
        self.manipulation_command_publisher.publish(command_msg)
        
        self.get_logger().info(f'Published grasp command for {object_id}')

    def image_callback(self, msg):
        """Process image for visualization of grasp planning"""
        # Convert to OpenCV format for potential visualization
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # In a real implementation, we might overlay grasp points on the image
            # for visualization/debugging purposes
            
            # For now, just log that we received an image
            self.get_logger().debug(f'Processed image for vision-guided grasping: {cv_image.shape}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    vision_grasper = VisionGuidedGrasping()
    
    try:
        rclpy.spin(vision_grasper)
    except KeyboardInterrupt:
        vision_grasper.get_logger().info('Shutting down vision-guided grasping...')
    finally:
        vision_grasper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Validation

Safety is paramount in manipulation systems:

### Safety Checks Implementation

```python
# manipulation_safety.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
import numpy as np

class ManipulationSafetyChecker(Node):
    def __init__(self):
        super().__init__('manipulation_safety_checker')
        
        # Subscriptions
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.force_torque_subscription = self.create_subscription(
            WrenchStamped,
            '/wrist_force_torque',
            self.force_torque_callback,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        self.manipulation_command_subscription = self.create_subscription(
            String,
            '/manipulation_commands',
            self.command_callback,
            10)
        
        # Publishers
        self.safety_status_publisher = self.create_publisher(
            String,
            '/manipulation_safety_status',
            10)
        
        # Initialize safety parameters
        self.joint_limits = {
            'shoulder_pan': (-2.0, 2.0),
            'shoulder_lift': (-2.0, 1.5),
            'elbow_flex': (-2.5, 0.0),
            'wrist_flex': (-1.5, 1.5),
            'wrist_roll': (-3.0, 3.0),
        }
        
        self.force_limits = {
            'max_normal_force': 100.0,  # Newtons
            'max_tangential_force': 50.0,  # Newtons
        }
        
        self.balance_threshold = 5.0  # Degrees from vertical
        
        # Robot state
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.current_force_torque = None
        self.current_orientation = None
        
        self.get_logger().info('Manipulation Safety Checker initialized')

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]
        
        # Check joint limits
        for joint_name, position in self.current_joint_positions.items():
            if joint_name in self.joint_limits:
                min_limit, max_limit = self.joint_limits[joint_name]
                if position < min_limit or position > max_limit:
                    self.publish_safety_violation(f'Joint {joint_name} out of limits: {position}')
                    self.emergency_stop()

    def force_torque_callback(self, msg):
        """Monitor force/torque for safety violations"""
        self.current_force_torque = msg
        
        # Check force limits
        force_magnitude = np.sqrt(
            msg.wrench.force.x**2 + 
            msg.wrench.force.y**2 + 
            msg.wrench.force.z**2
        )
        
        if force_magnitude > self.force_limits['max_normal_force']:
            self.publish_safety_violation(f'Excessive force detected: {force_magnitude}N')
            self.emergency_stop()

    def imu_callback(self, msg):
        """Monitor robot orientation for balance safety"""
        self.current_orientation = msg.orientation
        
        # Convert quaternion to Euler angles to check balance
        # Simplified calculation - in practice, use proper quaternion to Euler conversion
        import math
        siny_cosp = 2 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y)
        cosy_cosp = 1 - 2 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        siny_cosp = 2 * (msg.orientation.w * msg.orientation.y - msg.orientation.z * msg.orientation.x)
        pitch = math.asin(siny_cosp) if abs(siny_cosp) <= 1 else math.copysign(math.pi/2, siny_cosp)
        
        sinz_cosp = 2 * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z)
        cosz_cosp = 1 - 2 * (msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y)
        roll = math.atan2(sinz_cosp, cosz_cosp)
        
        # Check if robot is tilting too much (balance safety)
        tilt_angle = max(abs(roll), abs(pitch)) * 180.0 / math.pi  # Convert to degrees
        
        if tilt_angle > self.balance_threshold:
            self.publish_safety_violation(f'Robot balance compromised: tilt {tilt_angle:.2f}°')
            self.emergency_stop()

    def command_callback(self, msg):
        """Validate manipulation commands for safety"""
        try:
            command_data = json.loads(msg.data)
            
            # Validate command for potential safety issues
            if 'command_type' in command_data:
                if command_data['command_type'] == 'pick_and_place':
                    if 'object_pose' in command_data:
                        object_pose = command_data['object_pose']
                        # Check if object is in a safe reachable area
                        if not self.is_safe_reach_area(object_pose['position']):
                            self.publish_safety_violation(f'Object at {object_pose["position"]} is not in safe reach area')
                            # Instead of stopping, we could modify the command
                            self.modify_command_for_safety(msg)
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in command: {msg.data}')

    def is_safe_reach_area(self, position):
        """Check if a position is in the safe reach area"""
        # Define safe reach area (simplified for example)
        # In practice, this would be based on robot kinematics and workspace analysis
        
        # Safe reach area: x: -0.5 to 0.5m, y: -0.5 to 0.5m, z: 0.1 to 1.0m
        # relative to robot base
        safe_x_range = (-0.5, 0.5)
        safe_y_range = (-0.5, 0.5)
        safe_z_range = (0.1, 1.0)
        
        is_safe = (
            safe_x_range[0] <= position['x'] <= safe_x_range[1] and
            safe_y_range[0] <= position['y'] <= safe_y_range[1] and
            safe_z_range[0] <= position['z'] <= safe_z_range[1]
        )
        
        return is_safe

    def publish_safety_violation(self, violation_msg):
        """Publish safety violation message"""
        safety_msg = String()
        safety_msg.data = f'SAFETY_VIOLATION: {violation_msg}'
        self.safety_status_publisher.publish(safety_msg)

    def emergency_stop(self):
        """Send emergency stop command"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED due to safety violation')
        # In a real implementation, this would send emergency stop commands
        # to all robot controllers

    def modify_command_for_safety(self, original_msg):
        """Modify a command to make it safe"""
        # In a real implementation, this would adjust the command
        # to be within safe operating parameters
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_checker = ManipulationSafetyChecker()
    
    try:
        rclpy.spin(safety_checker)
    except KeyboardInterrupt:
        safety_checker.get_logger().info('Shutting down manipulation safety checker...')
    finally:
        safety_checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Manipulation Considerations

### Bimanual Manipulation

Humanoid robots often have two arms, enabling bimanual manipulation:

```python
# bimanual_manipulation.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json

class BimanualManipulationPlanner(Node):
    def __init__(self):
        super().__init__('bimanual_manipulation_planner')
        
        # Subscriptions
        self.bimanual_command_subscription = self.create_subscription(
            String,
            '/bimanual_commands',
            self.bimanual_command_callback,
            10)
        
        # Publishers
        self.left_arm_publisher = self.create_publisher(
            String,  # In practice, use JointTrajectory or similar
            '/left_arm/trajectory',
            10)
        
        self.right_arm_publisher = self.create_publisher(
            String,  # In practice, use JointTrajectory or similar
            '/right_arm/trajectory',
            10)
        
        self.coordinated_publisher = self.create_publisher(
            String,
            '/coordinated_manipulation',
            10)
        
        self.get_logger().info('Bimanual Manipulation Planner initialized')

    def bimanual_command_callback(self, msg):
        """Process bimanual manipulation commands"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('command_type')
            
            if command_type == 'bimanual_grasp':
                self.execute_bimanual_grasp(command_data)
            elif command_type == 'object_transfer':
                self.execute_object_transfer(command_data)
            elif command_type == 'assembly_task':
                self.execute_assembly_task(command_data)
            else:
                self.get_logger().warn(f'Unknown bimanual command: {command_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in bimanual command: {msg.data}')

    def execute_bimanual_grasp(self, command_data):
        """Execute bimanual grasp for large or heavy objects"""
        object_pose = command_data.get('object_pose')
        grasp_type = command_data.get('grasp_type', 'parallel_grip')
        
        # Calculate appropriate poses for both hands
        if grasp_type == 'parallel_grip':
            # Parallel grip: both hands on opposite sides of object
            left_hand_pose = self.calculate_left_hand_pose(object_pose)
            right_hand_pose = self.calculate_right_hand_pose(object_pose)
            
            # Execute coordinated grasp
            self.execute_coordinated_grasp(left_hand_pose, right_hand_pose)
        elif grasp_type == 'tripod_grip':
            # Tripod grip: one hand performs tripod grasp, other provides support
            primary_hand_pose = self.calculate_tripod_grasp_pose(object_pose)
            support_hand_pose = self.calculate_support_pose(object_pose)
            
            self.execute_tripod_grasp(primary_hand_pose, support_hand_pose)

    def calculate_left_hand_pose(self, object_pose):
        """Calculate appropriate pose for left hand"""
        # For parallel grip, left hand approaches from left side
        left_pose = Pose()
        left_pose.position.x = object_pose.position.x - 0.1  # 10cm to the left
        left_pose.position.y = object_pose.position.y
        left_pose.position.z = object_pose.position.z + 0.1  # 10cm above center
        left_pose.orientation = object_pose.orientation
        return left_pose

    def calculate_right_hand_pose(self, object_pose):
        """Calculate appropriate pose for right hand"""
        # For parallel grip, right hand approaches from right side
        right_pose = Pose()
        right_pose.position.x = object_pose.position.x + 0.1  # 10cm to the right
        right_pose.position.y = object_pose.position.y
        right_pose.position.z = object_pose.position.z + 0.1  # 10cm above center
        right_pose.orientation = object_pose.orientation
        return right_pose

    def execute_coordinated_grasp(self, left_pose, right_pose):
        """Execute coordinated grasp with both hands"""
        # In a real implementation, this would:
        # 1. Plan trajectories for both arms simultaneously
        # 2. Ensure no collisions between arms
        # 3. Execute synchronized motion
        # 4. Apply appropriate forces
        
        self.get_logger().info(f'Executing coordinated grasp: left={left_pose.position}, right={right_pose.position}')
        
        # Publish trajectories to both arms
        left_traj = self.generate_grasp_trajectory(left_pose, 'left')
        right_traj = self.generate_grasp_trajectory(right_pose, 'right')
        
        left_msg = String()
        left_msg.data = json.dumps(left_traj)
        self.left_arm_publisher.publish(left_msg)
        
        right_msg = String()
        right_msg.data = json.dumps(right_traj)
        self.right_arm_publisher.publish(right_msg)

    def generate_grasp_trajectory(self, target_pose, arm_side):
        """Generate a trajectory for grasping"""
        # This would generate a complete trajectory with multiple waypoints
        trajectory = {
            'arm': arm_side,
            'waypoints': [
                {
                    'pose': target_pose,
                    'time_from_start': 2.0  # 2 seconds to approach
                },
                {
                    'pose': target_pose,  # Maintain position
                    'time_from_start': 2.5,
                    'gripper_action': 'close'
                },
                {
                    'pose': self.calculate_lift_pose(target_pose),
                    'time_from_start': 4.0  # Lift after 4 seconds
                }
            ]
        }
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    bimanual_planner = BimanualManipulationPlanner()
    
    try:
        rclpy.spin(bimanual_planner)
    except KeyboardInterrupt:
        bimanual_planner.get_logger().info('Shutting down bimanual manipulation planner...')
    finally:
        bimanual_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Humanoid Manipulation

### 1. Whole-Body Coordination
- Consider the entire robot's pose when planning manipulation
- Coordinate arm movements with torso and potentially leg movements for balance
- Use whole-body controllers when available

### 2. Grasp Stability
- Plan grasps that provide stable contact with objects
- Consider object properties (weight, shape, material) in grasp planning
- Use appropriate grasp forces to prevent slippage without damaging objects

### 3. Safety First
- Implement comprehensive safety checks before executing manipulation
- Monitor forces and torques during manipulation
- Have emergency stop procedures ready

### 4. Integration with Navigation
- Plan manipulation tasks in coordination with navigation
- Consider the robot's position relative to objects
- Move robot closer to objects if needed for manipulation

## Troubleshooting Common Issues

### 1. Grasp Failures
- **Issue**: Robot fails to grasp objects consistently
- **Solutions**: 
  - Improve object pose estimation
  - Calibrate hand-eye coordination
  - Adjust grasp force parameters
  - Verify kinematic model accuracy

### 2. Collision Issues
- **Issue**: Robot collides with itself or environment during manipulation
- **Solutions**:
  - Improve collision checking algorithms
  - Use trajectory optimization with collision avoidance
  - Verify environment models are accurate

### 3. Balance Problems
- **Issue**: Robot loses balance during manipulation
- **Solutions**:
  - Implement whole-body control
  - Adjust center of mass during manipulation
  - Use appropriate stance adjustments

## Summary

In this section, we've explored manipulation capabilities for humanoid robots, including:

- Hand kinematics and grasp planning
- Manipulation action sequences for complex tasks
- Vision-guided manipulation for adaptive behavior
- Safety considerations and validation procedures
- Bimanual manipulation for complex tasks
- Best practices for humanoid-specific manipulation

Manipulation is a critical capability for humanoid robots, enabling them to interact with the environment in human-like ways. The combination of computer vision, AI planning, and precise control enables robots to perform complex manipulation tasks that require dexterity and adaptability.

In the next section, we'll explore how to integrate these manipulation capabilities with the language understanding and action execution components to create complete Vision-Language-Action systems.