# Language-to-Action Mapping

Language-to-Action (LTA) mapping is the critical bridge between natural language understanding and robot execution in Vision-Language-Action (VLA) systems. This section covers how to convert natural language commands into executable robot actions, ensuring that humanoid robots can accurately interpret and respond to human instructions.

## Learning Objectives

By the end of this section, you will be able to:
- Map natural language commands to specific robot actions
- Implement intent recognition for robotic commands
- Create action grammars and parsers for robotic systems
- Design safety checks and validation for language-to-action conversion
- Handle ambiguous or complex language commands
- Implement fallback mechanisms when language interpretation fails
- Optimize the language-to-action pipeline for real-time performance

## Introduction to Language-to-Action Mapping

Language-to-Action mapping involves converting human natural language commands into structured robot actions. This process includes:

1. **Intent Recognition**: Determining what the user wants the robot to do
2. **Entity Extraction**: Identifying objects, locations, and parameters in the command
3. **Action Mapping**: Converting the intent and entities into executable robot actions
4. **Validation**: Ensuring the actions are safe and feasible
5. **Execution**: Sending the actions to the robot control system

For humanoid robots, LTA mapping is particularly challenging because:
- Actions must consider balance and stability
- Manipulation actions must account for humanoid kinematics
- Navigation must consider bipedal locomotion constraints
- Multiple modalities (vision, hearing, touch) must be integrated

## Intent Recognition Systems

### Rule-Based Intent Recognition

Simple rule-based systems can handle common commands:

```python
# intent_recognizer.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
import re
from typing import Dict, List, Tuple, Optional

class IntentRecognizer(Node):
    def __init__(self):
        super().__init__('intent_recognizer')
        
        # Subscribe to natural language commands
        self.command_subscription = self.create_subscription(
            String,
            '/natural_language_commands',
            self.command_callback,
            10)
        
        # Publisher for recognized intents
        self.intent_publisher = self.create_publisher(
            String,  # In practice, this would be a custom message type
            '/parsed_intents',
            10)
        
        # Define command patterns
        self.command_patterns = [
            # Movement commands
            {
                'pattern': r'(?:move|go|drive|walk|navigate|travel)\s+(forward|backward|ahead|back|left|right|up|down)\s*(\d*\.?\d*)\s*(meter|meters|cm|centimeters|step|steps|foot|feet)?',
                'intent': 'MOVE_DIRECTION_DISTANCE',
                'extractors': ['direction', 'distance', 'unit']
            },
            {
                'pattern': r'(?:turn|rotate|pivot)\s+(left|right|around)\s*(\d*\.?\d*)\s*(degree|degrees|deg|turn|turns)?',
                'intent': 'TURN_DIRECTION_ANGLE',
                'extractors': ['direction', 'angle', 'unit']
            },
            {
                'pattern': r'(?:move|go|navigate)\s+to\s+(?:the\s+)?(\w+)',
                'intent': 'NAVIGATE_TO_LOCATION',
                'extractors': ['location']
            },
            # Manipulation commands
            {
                'pattern': r'(?:pick up|grasp|grab|take|lift|collect|get)\s+(?:the\s+)?(\w+(?:\s+\w+)*)',
                'intent': 'PICK_UP_OBJECT',
                'extractors': ['object']
            },
            {
                'pattern': r'(?:place|put|set|drop|release|place down)\s+(?:the\s+)?(\w+(?:\s+\w+)*)\s+(?:on|at|in)\s+(?:the\s+)?(\w+(?:\s+\w+)*)',
                'intent': 'PLACE_OBJECT_AT_LOCATION',
                'extractors': ['object', 'location']
            },
            {
                'pattern': r'(?:bring|carry|transport|move)\s+(?:the\s+)?(\w+(?:\s+\w+)*)\s+to\s+(?:the\s+)?(\w+(?:\s+\w+)*)',
                'intent': 'TRANSPORT_OBJECT_TO_LOCATION',
                'extractors': ['object', 'destination']
            },
            # Interaction commands
            {
                'pattern': r'(?:speak|say|tell|announce)\s+"([^"]+)"',
                'intent': 'SPEAK_TEXT',
                'extractors': ['text']
            },
            {
                'pattern': r'(?:stop|halt|pause|freeze|wait)\s*(\d*\.?\d*)\s*(second|seconds|minute|minutes)?',
                'intent': 'STOP_OR_WAIT',
                'extractors': ['duration', 'unit']
            }
        ]
        
        self.get_logger().info('Intent Recognizer initialized')

    def command_callback(self, msg):
        """Process natural language command and extract intent"""
        command_text = msg.data.lower()
        self.get_logger().info(f'Processing command: {command_text}')
        
        # Attempt to match command to patterns
        intent_result = self.match_command_to_intent(command_text)
        
        if intent_result:
            intent_type, entities = intent_result
            self.get_logger().info(f'Matched intent: {intent_type} with entities: {entities}')
            
            # Publish the parsed intent
            intent_msg = String()
            intent_msg.data = f'{intent_type}|{json.dumps(entities)}'
            self.intent_publisher.publish(intent_msg)
        else:
            self.get_logger().warn(f'Could not parse command: {command_text}')
            
            # Request clarification
            clarification_msg = String()
            clarification_msg.data = f'CLARIFICATION_NEEDED|{command_text}'
            self.intent_publisher.publish(clarification_msg)

    def match_command_to_intent(self, command: str) -> Optional[Tuple[str, Dict[str, str]]]:
        """Match command to intent using regex patterns"""
        for pattern_config in self.command_patterns:
            match = re.search(pattern_config['pattern'], command)
            if match:
                # Extract entities based on the pattern groups
                entities = {}
                groups = match.groups()
                
                for i, extractor in enumerate(pattern_config['extractors']):
                    if i < len(groups) and groups[i]:
                        entities[extractor] = groups[i]
                
                return pattern_config['intent'], entities
        
        return None

def main(args=None):
    rclpy.init(args=args)
    recognizer = IntentRecognizer()
    
    try:
        rclpy.spin(recognizer)
    except KeyboardInterrupt:
        recognizer.get_logger().info('Shutting down intent recognizer...')
    finally:
        recognizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Machine Learning-Based Intent Recognition

For more complex language understanding, we can use machine learning models:

```python
# ml_intent_recognizer.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import pipeline
import json
from typing import Dict, Any

class MLIntentRecognizer(Node):
    def __init__(self):
        super().__init__('ml_intent_recognizer')
        
        # Subscribe to natural language commands
        self.command_subscription = self.create_subscription(
            String,
            '/natural_language_commands',
            self.command_callback,
            10)
        
        # Publisher for recognized intents
        self.intent_publisher = self.create_publisher(
            String,
            '/ml_parsed_intents',
            10)
        
        # Initialize transformer pipeline for zero-shot classification
        # This example uses a zero-shot classifier that can identify intents without training
        self.classifier = pipeline(
            "zero-shot-classification", 
            model="facebook/bart-large-mnli"
        )
        
        # Define possible intents for robotic commands
        self.robot_intents = [
            "movement command",
            "manipulation command", 
            "navigation command",
            "interaction command",
            "information request",
            "system command",
            "stop command"
        ]
        
        self.get_logger().info('ML Intent Recognizer initialized')

    def command_callback(self, msg):
        """Process command using ML-based intent recognition"""
        command_text = msg.data
        self.get_logger().info(f'Processing command with ML: {command_text}')
        
        try:
            # Classify the command intent
            result = self.classifier(command_text, self.robot_intents)
            
            # Determine the most likely intent
            top_intent = result['labels'][0]
            confidence = result['scores'][0]
            
            # If confidence is too low, request clarification
            if confidence < 0.7:
                self.get_logger().warn(f'Low confidence ({confidence:.2f}) for intent: {top_intent}')
                # In practice, you might want to request clarification from the user
                return
            
            # Extract additional information using NLP techniques
            entities = self.extract_entities(command_text)
            
            # Create intent result
            intent_result = {
                'intent': top_intent,
                'confidence': confidence,
                'entities': entities,
                'original_command': command_text
            }
            
            # Publish the result
            intent_msg = String()
            intent_msg.data = json.dumps(intent_result)
            self.intent_publisher.publish(intent_msg)
            
            self.get_logger().info(f'ML recognized intent: {top_intent} (confidence: {confidence:.2f})')
            
        except Exception as e:
            self.get_logger().error(f'Error in ML intent recognition: {e}')

    def extract_entities(self, command: str) -> Dict[str, str]:
        """Extract entities like objects, locations, and parameters"""
        entities = {}
        
        # Simple entity extraction using regex (in practice, use NER models)
        # Extract numbers
        numbers = re.findall(r'(\d+\.?\d*)', command)
        if numbers:
            entities['numbers'] = numbers
        
        # Extract potential objects (nouns)
        # This is a simplified approach - in practice, use proper NLP
        potential_objects = re.findall(r'\b(?:the\s+)?(\w+)\b', command)
        objects_of_interest = [obj for obj in potential_objects 
                              if obj not in ['the', 'a', 'an', 'to', 'from', 'with', 'on', 'at', 'in']]
        if objects_of_interest:
            entities['potential_objects'] = objects_of_interest
        
        # Extract locations (common location words)
        location_words = ['kitchen', 'living room', 'bedroom', 'office', 'hallway', 'door', 'table', 'chair']
        found_locations = [word for word in location_words if word in command]
        if found_locations:
            entities['locations'] = found_locations
        
        return entities

def main(args=None):
    rclpy.init(args=args)
    ml_recognizer = MLIntentRecognizer()
    
    try:
        rclpy.spin(ml_recognizer)
    except KeyboardInterrupt:
        ml_recognizer.get_logger().info('Shutting down ML intent recognizer...')
    finally:
        ml_recognizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action Mapping and Grammar Systems

### Action Grammar Definition

Define a grammar for robotic actions that can be parsed from natural language:

```python
# action_grammar.py
from typing import Dict, List, Any, Union
import json

class RobotActionGrammar:
    def __init__(self):
        # Define the action grammar structure
        self.grammar = {
            'base_actions': {
                'move': {
                    'patterns': [
                        'move_forward_distance',
                        'move_backward_distance', 
                        'move_to_location',
                        'move_in_direction_angle'
                    ],
                    'parameters': ['distance', 'angle', 'direction', 'location']
                },
                'rotate': {
                    'patterns': [
                        'turn_left_angle',
                        'turn_right_angle',
                        'rotate_in_place_angle'
                    ],
                    'parameters': ['angle', 'direction']
                },
                'manipulate': {
                    'patterns': [
                        'pick_up_object',
                        'place_object_at_location',
                        'grasp_object',
                        'release_object'
                    ],
                    'parameters': ['object', 'location', 'pose']
                },
                'navigate': {
                    'patterns': [
                        'navigate_to_location',
                        'go_to_location',
                        'travel_to_location'
                    ],
                    'parameters': ['location', 'waypoints']
                },
                'communicate': {
                    'patterns': [
                        'speak_text',
                        'listen_to_user',
                        'display_message'
                    ],
                    'parameters': ['text', 'language']
                }
            },
            'composite_actions': {
                'transport_object': [
                    'navigate_to_object_location',
                    'pick_up_object',
                    'navigate_to_destination',
                    'place_object_at_destination'
                ],
                'inspect_area': [
                    'navigate_to_location',
                    'turn_to_face_object',
                    'take_picture',
                    'analyze_image'
                ],
                'follow_person': [
                    'detect_person',
                    'maintain_distance',
                    'adjust_direction'
                ]
            }
        }
    
    def parse_command_to_action(self, intent: str, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Parse intent and entities into executable action"""
        action_mapping = {
            'MOVE_DIRECTION_DISTANCE': self.map_move_direction_distance,
            'TURN_DIRECTION_ANGLE': self.map_turn_direction_angle,
            'NAVIGATE_TO_LOCATION': self.map_navigate_to_location,
            'PICK_UP_OBJECT': self.map_pick_up_object,
            'PLACE_OBJECT_AT_LOCATION': self.map_place_object_at_location,
            'SPEAK_TEXT': self.map_speak_text,
            'STOP_OR_WAIT': self.map_stop_or_wait
        }
        
        if intent in action_mapping:
            return action_mapping[intent](entities)
        else:
            return self.create_unknown_action(intent, entities)
    
    def map_move_direction_distance(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Map movement command to robot action"""
        direction = entities.get('direction', 'forward')
        distance_str = entities.get('distance', '1')
        unit = entities.get('unit', 'meter')
        
        # Convert distance to meters
        distance = float(distance_str)
        if unit and unit.startswith('c'):  # centimeter
            distance = distance / 100.0
        elif unit and unit.startswith('f'):  # foot/feet
            distance = distance * 0.3048  # Convert feet to meters
        
        # Map direction to robot velocities
        if direction in ['forward', 'ahead']:
            linear_vel = 0.5  # m/s
            angular_vel = 0.0
        elif direction in ['backward', 'back']:
            linear_vel = -0.5  # m/s
            angular_vel = 0.0
        elif direction in ['left', 'leftward']:
            linear_vel = 0.0
            angular_vel = 0.5  # rad/s
        elif direction in ['right', 'rightward']:
            linear_vel = 0.0
            angular_vel = -0.5  # rad/s
        else:
            linear_vel = 0.0
            angular_vel = 0.0
        
        # Calculate duration based on distance and speed
        if abs(linear_vel) > 0.001:  # Moving linearly
            duration = distance / abs(linear_vel)
        else:  # Rotating
            angle = distance  # For rotation, distance parameter is angle
            duration = abs(angle) / abs(angular_vel) if abs(angular_vel) > 0.001 else 0
        
        return {
            'action_type': 'move_base',
            'parameters': {
                'linear_velocity': linear_vel,
                'angular_velocity': angular_vel,
                'duration': duration
            },
            'description': f'Move {direction} by {distance} meters',
            'preconditions': ['robot_is_stationary', 'path_is_clear'],
            'postconditions': ['robot_moved_to_new_position']
        }
    
    def map_turn_direction_angle(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Map turn command to robot action"""
        direction = entities.get('direction', 'left')
        angle_str = entities.get('angle', '90')
        unit = entities.get('unit', 'degree')
        
        # Convert angle to radians
        angle = float(angle_str)
        if unit in ['degree', 'degrees', 'deg']:
            angle = angle * 3.14159 / 180.0  # Convert degrees to radians
        
        # Map direction to angular velocity
        if direction in ['left', 'counterclockwise']:
            angular_vel = 0.5  # rad/s
        elif direction in ['right', 'clockwise']:
            angular_vel = -0.5  # rad/s
        else:
            angular_vel = 0.5 if direction == 'around' else 0.0  # Default to left turn
        
        # Calculate duration
        duration = abs(angle) / abs(angular_vel)
        
        return {
            'action_type': 'rotate_base',
            'parameters': {
                'angular_velocity': angular_vel,
                'angle': angle,
                'duration': duration
            },
            'description': f'Turn {direction} by {angle_str} degrees',
            'preconditions': ['robot_is_stationary', 'surroundings_clear'],
            'postconditions': ['robot_rotated_by_angle']
        }
    
    def map_navigate_to_location(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Map navigation command to robot action"""
        location = entities.get('location', 'destination')
        
        return {
            'action_type': 'navigate_to_pose',
            'parameters': {
                'location_name': location
            },
            'description': f'Navigate to {location}',
            'preconditions': ['location_known', 'path_not_blocked'],
            'postconditions': ['robot_at_destination']
        }
    
    def map_pick_up_object(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Map pick up command to robot action"""
        object_name = entities.get('object', 'object')
        
        return {
            'action_type': 'pick_object',
            'parameters': {
                'object_name': object_name
            },
            'description': f'Pick up {object_name}',
            'preconditions': ['object_detected', 'object_reachable', 'gripper_free'],
            'postconditions': ['object_grasped', 'gripper_occupied']
        }
    
    def map_place_object_at_location(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Map place command to robot action"""
        object_name = entities.get('object', 'object')
        location = entities.get('location', 'surface')
        
        return {
            'action_type': 'place_object',
            'parameters': {
                'object_name': object_name,
                'location': location
            },
            'description': f'Place {object_name} at {location}',
            'preconditions': ['object_grasped', 'location_reachable', 'location_clear'],
            'postconditions': ['object_placed', 'gripper_free']
        }
    
    def map_speak_text(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Map speak command to robot action"""
        text = entities.get('text', 'Hello')
        
        return {
            'action_type': 'speak',
            'parameters': {
                'text': text
            },
            'description': f'Speak: {text}',
            'preconditions': ['speech_system_ready'],
            'postconditions': ['text_spoken']
        }
    
    def map_stop_or_wait(self, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Map stop/wait command to robot action"""
        duration_str = entities.get('duration', '0')
        unit = entities.get('unit', 'second')
        
        duration = float(duration_str)
        if unit in ['minute', 'minutes']:
            duration = duration * 60.0  # Convert minutes to seconds
        
        if duration > 0:
            action_type = 'wait'
            description = f'Wait for {duration} seconds'
        else:
            action_type = 'stop'
            description = 'Stop current motion'
        
        return {
            'action_type': action_type,
            'parameters': {
                'duration': duration
            },
            'description': description,
            'preconditions': [],
            'postconditions': ['motion_stopped']
        }
    
    def create_unknown_action(self, intent: str, entities: Dict[str, Any]) -> Dict[str, Any]:
        """Create a fallback action for unknown intents"""
        return {
            'action_type': 'unknown',
            'parameters': {
                'original_intent': intent,
                'entities': entities
            },
            'description': f'Unknown command: {intent}',
            'preconditions': [],
            'postconditions': [],
            'requires_clarification': True
        }

# Example usage
# grammar = RobotActionGrammar()
# action = grammar.parse_command_to_action('MOVE_DIRECTION_DISTANCE', {'direction': 'forward', 'distance': '2', 'unit': 'meters'})