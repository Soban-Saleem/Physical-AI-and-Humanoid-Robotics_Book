# Module 4 Exercises: Vision-Language-Action (VLA)

This section provides hands-on exercises to reinforce the concepts learned in Module 4. These exercises will help you practice and apply the Vision-Language-Action concepts covered in the previous sections.

## Exercise 1: Implement Voice Command Processing

### Objective
Create a complete voice processing pipeline that captures voice commands, processes them with an LLM, and converts them to robot actions.

### Requirements
- Implement a voice capture system using PyAudio or similar
- Integrate with OpenAI Whisper or a similar speech-to-text service
- Connect to an LLM for command interpretation
- Convert interpreted commands to robot action sequences
- Validate actions before execution

### Steps to Complete
1. Create a ROS 2 node for voice capture
2. Implement speech-to-text conversion
3. Connect to an LLM for command interpretation
4. Map interpreted commands to robot actions
5. Implement action validation and safety checks
6. Test with various voice commands

### Solution Approach

First, create the voice processing node:

```python
# voice_command_processor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import pyaudio
import wave
import threading
import queue
import numpy as np
import openai
import json
import time

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')
        
        # Initialize audio processing parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.record_seconds = 5
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)
        self.action_publisher = self.create_publisher(String, '/robot_actions', 10)
        
        # Initialize audio buffer
        self.audio_buffer = queue.Queue()
        self.listening = False
        self.recording = False
        
        # Voice activity detection parameters
        self.energy_threshold = 1000
        self.silence_duration = 1.0
        self.last_voice_time = time.time()
        
        # Timer for continuous listening
        self.listen_timer = self.create_timer(0.1, self.check_voice_activity)
        
        self.get_logger().info('Voice Command Processor initialized')

    def check_voice_activity(self):
        """Continuously check for voice activity"""
        # Open stream to read audio
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        # Read a chunk of audio
        data = stream.read(self.chunk, exception_on_overflow=False)
        audio_data = np.frombuffer(data, dtype=np.int16)
        
        # Calculate energy (volume) of the audio chunk
        energy = np.sum(audio_data ** 2) / len(audio_data)
        
        # Check if energy exceeds threshold (indicating voice activity)
        if energy > self.energy_threshold and not self.recording:
            self.start_recording()
        elif energy <= self.energy_threshold and self.recording:
            # Check if enough silence has passed to stop recording
            if time.time() - self.last_voice_time > self.silence_duration:
                self.stop_recording()
        
        if self.recording:
            self.last_voice_time = time.time()
        
        stream.stop_stream()
        stream.close()

    def start_recording(self):
        """Start recording when voice activity is detected"""
        self.get_logger().info('Starting voice recording...')
        self.recording = True
        self.frames = []
        
        # Record in a separate thread to avoid blocking
        self.recording_thread = threading.Thread(target=self.record_audio)
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def record_audio(self):
        """Record audio until silence is detected"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        start_time = time.time()
        while self.recording and (time.time() - start_time) < self.record_seconds:
            data = stream.read(self.chunk, exception_on_overflow=False)
            self.frames.append(data)
            
            # Check for silence to stop early
            audio_data = np.frombuffer(data, dtype=np.int16)
            energy = np.sum(audio_data ** 2) / len(audio_data)
            
            if energy <= self.energy_threshold:
                if time.time() - self.last_voice_time > self.silence_duration:
                    break
            else:
                self.last_voice_time = time.time()
        
        stream.stop_stream()
        stream.close()
        
        # Save recorded audio to temporary file for processing
        temp_filename = f"/tmp/recording_{int(time.time())}.wav"
        wf = wave.open(temp_filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.audio.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(self.frames))
        wf.close()
        
        self.get_logger().info(f'Recording saved to {temp_filename}')
        
        # Process the recorded audio
        self.process_recorded_audio(temp_filename)

    def stop_recording(self):
        """Stop recording when silence is detected"""
        self.get_logger().info('Stopping voice recording...')
        self.recording = False

    def process_recorded_audio(self, filename):
        """Process recorded audio file to extract text and convert to actions"""
        try:
            # In a real implementation, this would use Whisper or similar
            # For this example, we'll simulate the transcription
            transcribed_text = self.simulate_transcription(filename)
            
            if transcribed_text:
                self.get_logger().info(f'Transcribed: {transcribed_text}')
                
                # Interpret the command with LLM
                action_sequence = self.interpret_command_with_llm(transcribed_text)
                
                if action_sequence:
                    # Validate actions before execution
                    if self.validate_actions(action_sequence):
                        # Execute the action sequence
                        self.execute_action_sequence(action_sequence)
                    else:
                        self.get_logger().warn('Actions failed validation')
                else:
                    self.get_logger().warn('Could not interpret command')
            else:
                self.get_logger().warn('Could not transcribe audio')
                
        except Exception as e:
            self.get_logger().error(f'Error processing recorded audio: {e}')
        finally:
            # Clean up temporary file
            import os
            if os.path.exists(filename):
                os.remove(filename)

    def simulate_transcription(self, filename):
        """Simulate Whisper transcription (replace with actual Whisper API call)"""
        # In a real implementation, this would call the Whisper API
        # For simulation, return a sample command
        return "move forward 1 meter and pick up the red cup"

    def interpret_command_with_llm(self, command_text):
        """Use LLM to interpret command and generate action sequence"""
        try:
            # In a real implementation, this would call an LLM API
            # For this example, we'll use a simple rule-based approach
            
            # This is a simplified example - in reality, you'd use OpenAI or similar
            # to generate structured action sequences from natural language
            if "move forward" in command_text:
                distance = self.extract_distance(command_text)
                return [{
                    'action_type': 'move_forward',
                    'parameters': {'distance': distance},
                    'description': f'Move forward {distance} meters'
                }]
            elif "pick up" in command_text or "grasp" in command_text:
                object_name = self.extract_object(command_text)
                return [{
                    'action_type': 'pick_object',
                    'parameters': {'object_name': object_name},
                    'description': f'Pick up {object_name}'
                }]
            elif "turn left" in command_text:
                angle = self.extract_angle(command_text)
                return [{
                    'action_type': 'turn_left',
                    'parameters': {'angle': angle},
                    'description': f'Turn left {angle} degrees'
                }]
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error interpreting command: {e}')
            return None

    def extract_distance(self, command):
        """Extract distance from command (simplified implementation)"""
        import re
        match = re.search(r'(\d+\.?\d*)\s*(meter|meters|cm|centimeters)', command)
        if match:
            distance = float(match.group(1))
            unit = match.group(2)
            if unit.startswith('c'):  # centimeters
                distance = distance / 100.0
            return distance
        return 1.0  # default distance

    def extract_object(self, command):
        """Extract object name from command (simplified implementation)"""
        # Look for object after "pick up" or "grasp"
        if "pick up" in command:
            start_idx = command.find("pick up") + len("pick up")
        elif "grasp" in command:
            start_idx = command.find("grasp") + len("grasp")
        elif "get" in command:
            start_idx = command.find("get") + len("get")
        else:
            return "object"
        
        object_part = command[start_idx:].strip()
        # Extract the first noun phrase
        words = object_part.split()
        object_name = ""
        for word in words:
            if word in ["the", "a", "an", "and", "then"]:
                continue
            # Stop at words that indicate end of object name
            if word in ["and", "then", "after", "before"]:
                break
            object_name += word + " "
        
        return object_name.strip() or "object"

    def extract_angle(self, command):
        """Extract angle from command (simplified implementation)"""
        import re
        match = re.search(r'(\d+\.?\d*)\s*(degree|degrees|deg)', command)
        if match:
            return float(match.group(1))
        return 90.0  # default angle

    def validate_actions(self, action_sequence):
        """Validate action sequence before execution"""
        # Check that all actions are valid types
        valid_actions = {
            'move_forward', 'move_backward', 'turn_left', 'turn_right', 
            'pick_object', 'place_object', 'speak', 'wait', 'navigate_to'
        }
        
        for action in action_sequence:
            if action['action_type'] not in valid_actions:
                self.get_logger().error(f'Invalid action type: {action["action_type"]}')
                return False
        
        # Check that parameters are appropriate
        for action in action_sequence:
            if action['action_type'] in ['move_forward', 'move_backward']:
                if not isinstance(action['parameters'].get('distance', 0), (int, float)):
                    self.get_logger().error(f'Invalid distance parameter: {action["parameters"]}')
                    return False
            elif action['action_type'] in ['turn_left', 'turn_right']:
                if not isinstance(action['parameters'].get('angle', 0), (int, float)):
                    self.get_logger().error(f'Invalid angle parameter: {action["parameters"]}')
                    return False
        
        return True

    def execute_action_sequence(self, action_sequence):
        """Execute the validated action sequence"""
        for action in action_sequence:
            self.execute_single_action(action)

    def execute_single_action(self, action):
        """Execute a single action"""
        action_type = action['action_type']
        parameters = action['parameters']
        
        if action_type == 'move_forward':
            self.move_forward(parameters['distance'])
        elif action_type == 'move_backward':
            self.move_backward(parameters['distance'])
        elif action_type == 'turn_left':
            self.turn_left(parameters['angle'])
        elif action_type == 'turn_right':
            self.turn_right(parameters['angle'])
        elif action_type == 'pick_object':
            self.pick_object(parameters['object_name'])
        elif action_type == 'speak':
            self.speak(parameters['text'])

    def move_forward(self, distance):
        """Move robot forward by specified distance"""
        cmd = Twist()
        cmd.linear.x = 0.5  # m/s
        cmd.angular.z = 0.0
        
        # Calculate duration based on distance and speed
        duration = distance / 0.5
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(cmd)
            time.sleep(0.1)
        
        # Stop robot
        cmd.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    processor = VoiceCommandProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Shutting down voice command processor...')
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 2: Cognitive Planning with LLMs

### Objective
Implement cognitive planning using Large Language Models to decompose complex tasks into executable robot actions.

### Requirements
- Create a node that receives high-level commands
- Use an LLM to decompose commands into action sequences
- Validate action sequences for safety and feasibility
- Integrate with ROS 2 action execution system
- Handle ambiguous or complex commands

### Steps to Complete
1. Create an LLM-based task decomposition node
2. Implement command parsing and interpretation
3. Generate structured action sequences
4. Implement validation and safety checks
5. Test with complex, multi-step commands

### Solution Approach

```python
# cognitive_planner.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json
import re

class CognitivePlanner(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        
        # Subscription for high-level commands
        self.command_subscription = self.create_subscription(
            String,
            '/high_level_commands',
            self.command_callback,
            10)
        
        # Publisher for action sequences
        self.action_sequence_publisher = self.create_publisher(
            String,  # In practice, use a custom action sequence message
            '/action_sequences',
            10)
        
        # Publisher for clarification requests
        self.clarification_publisher = self.create_publisher(
            String,
            '/clarification_requests',
            10)
        
        # Initialize OpenAI API
        # openai.api_key = self.get_parameter_or_set_default('openai_api_key', '')
        
        # Robot capabilities and environment knowledge
        self.robot_capabilities = [
            'move_forward', 'move_backward', 'turn_left', 'turn_right',
            'pick_object', 'place_object', 'navigate_to', 'speak', 'wait'
        ]
        
        self.environment_knowledge = {
            'objects': ['cup', 'ball', 'box', 'chair', 'table'],
            'locations': ['kitchen', 'living room', 'bedroom', 'office'],
            'reachable_distance': 1.0  # meters
        }
        
        self.get_logger().info('Cognitive Planner initialized')

    def command_callback(self, msg):
        """Process high-level command and generate action sequence"""
        command = msg.data
        self.get_logger().info(f'Processing high-level command: {command}')
        
        # Decompose command into action sequence using LLM
        action_sequence = self.decompose_command_with_llm(command)
        
        if action_sequence:
            # Validate the action sequence
            if self.validate_action_sequence(action_sequence):
                # Publish the validated action sequence
                action_msg = String()
                action_msg.data = json.dumps({
                    'command': command,
                    'action_sequence': action_sequence,
                    'timestamp': self.get_clock().now().nanoseconds
                })
                self.action_sequence_publisher.publish(action_msg)
                
                self.get_logger().info(f'Published action sequence: {action_sequence}')
            else:
                self.get_logger().warn('Generated action sequence failed validation')
        else:
            self.get_logger().warn(f'Could not decompose command: {command}')

    def decompose_command_with_llm(self, command):
        """Decompose command into action sequence using LLM"""
        try:
            # In a real implementation, this would call an LLM API
            # For this example, we'll use a simplified approach
            
            # Create a prompt for the LLM
            prompt = f"""
            You are a cognitive planner for a humanoid robot. Decompose the following high-level command into a sequence of executable actions.

            Command: "{command}"

            Robot Capabilities: {', '.join(self.robot_capabilities)}
            Environment Objects: {', '.join(self.environment_knowledge['objects'])}
            Environment Locations: {', '.join(self.environment_knowledge['locations'])}

            Please provide the action sequence in JSON format with the following structure:
            {{
                "actions": [
                    {{
                        "action_type": "...",
                        "parameters": {{...}},
                        "description": "..."
                    }}
                ]
            }}

            Each action should be:
            1. Feasible given the robot's capabilities
            2. Safe to execute
            3. Sequentially ordered
            4. Include any necessary preconditions or checks
            """
            
            # For this example, we'll simulate the LLM response
            # In reality, this would be an API call to OpenAI or similar
            return self.simulate_llm_response(command)
            
        except Exception as e:
            self.get_logger().error(f'Error in LLM command decomposition: {e}')
            return None

    def simulate_llm_response(self, command):
        """Simulate LLM response for command decomposition"""
        # This is a simplified simulation - in reality, this would be an LLM API response
        import re
        
        action_sequence = []
        
        if "bring me" in command or "get me" in command:
            # Extract object and location
            object_match = re.search(r'(?:bring me|get me) (?:the )?(.+?)(?: from| to|$)', command)
            object_name = object_match.group(1).strip() if object_match else "object"
            
            # Create action sequence for fetching object
            action_sequence = [
                {
                    "action_type": "locate_object",
                    "parameters": {"object_name": object_name},
                    "description": f"Locate the {object_name}"
                },
                {
                    "action_type": "navigate_to",
                    "parameters": {"target": f"near_{object_name}"},
                    "description": f"Navigate to the {object_name}"
                },
                {
                    "action_type": "pick_object",
                    "parameters": {"object_name": object_name},
                    "description": f"Pick up the {object_name}"
                },
                {
                    "action_type": "navigate_to",
                    "parameters": {"target": "user_location"},
                    "description": "Navigate to user location"
                },
                {
                    "action_type": "place_object",
                    "parameters": {"object_name": object_name, "location": "user_handoff"},
                    "description": f"Place the {object_name} for user"
                }
            ]
        elif "go to" in command or "navigate to" in command:
            # Extract location
            location_match = re.search(r'(?:go to|navigate to) (?:the )?(.+)', command)
            location = location_match.group(1).strip() if location_match else "destination"
            
            action_sequence = [
                {
                    "action_type": "navigate_to",
                    "parameters": {"target": location},
                    "description": f"Navigate to {location}"
                }
            ]
        elif "move" in command and ("forward" in command or "backward" in command):
            # Extract distance
            distance_match = re.search(r'(\d+\.?\d*)\s*(meter|meters|cm|centimeters)', command)
            distance = float(distance_match.group(1)) if distance_match else 1.0
            
            direction = "forward" if "forward" in command else "backward"
            
            action_sequence = [
                {
                    "action_type": f"move_{direction}",
                    "parameters": {"distance": distance},
                    "description": f"Move {direction} by {distance} meters"
                }
            ]
        else:
            # For unrecognized commands, request clarification
            clarification_msg = String()
            clarification_msg.data = f"I don't understand how to execute: {command}. Can you rephrase?"
            self.clarification_publisher.publish(clarification_msg)
            return None
        
        return action_sequence

    def validate_action_sequence(self, action_sequence):
        """Validate action sequence for safety and feasibility"""
        if not action_sequence or not isinstance(action_sequence, list):
            self.get_logger().error('Action sequence is not a valid list')
            return False
        
        for action in action_sequence:
            if not isinstance(action, dict):
                self.get_logger().error(f'Invalid action format: {action}')
                return False
            
            if 'action_type' not in action:
                self.get_logger().error(f'Missing action_type in action: {action}')
                return False
            
            action_type = action['action_type']
            if action_type not in self.robot_capabilities:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                # We'll allow unknown actions for flexibility, but log them
        
        # Additional validation could include:
        # - Checking that navigation targets exist
        # - Ensuring object manipulation actions are preceded by detection
        # - Verifying that action parameters are within robot limits
        
        return True

def main(args=None):
    rclpy.init(args=args)
    planner = CognitivePlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Shutting down cognitive planner...')
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 3: Language-to-Action Mapping System

### Objective
Create a comprehensive system that maps natural language commands to specific robot actions, handling various command types and edge cases.

### Requirements
- Implement intent recognition for different command types
- Extract entities (objects, locations, parameters) from commands
- Map intents to appropriate robot actions
- Handle command variations and synonyms
- Implement fallback strategies for unrecognized commands

### Steps to Complete
1. Create intent recognition system
2. Implement entity extraction
3. Map intents to actions
4. Add synonym and variation handling
5. Implement fallback and clarification mechanisms
6. Test with various command formulations

### Solution Approach

```python
# language_action_mapper.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import json

class LanguageActionMapper(Node):
    def __init__(self):
        super().__init__('language_action_mapper')
        
        # Subscription for natural language commands
        self.command_subscription = self.create_subscription(
            String,
            '/natural_language_commands',
            self.command_callback,
            10)
        
        # Publisher for mapped actions
        self.action_publisher = self.create_publisher(
            String,  # In practice, use a custom action message type
            '/mapped_actions',
            10)
        
        # Publisher for clarification requests
        self.clarification_publisher = self.create_publisher(
            String,
            '/clarification_requests',
            10)
        
        # Define command patterns and their corresponding actions
        self.command_patterns = [
            # Movement patterns
            {
                'pattern': r'(?:move|go|drive|navigate|walk)\s+(forward|backward|ahead|back|left|right|up|down)\s*(\d*\.?\d*)\s*(meter|meters|cm|centimeters|step|steps)?',
                'intent': 'MOVE_DIRECTION_DISTANCE',
                'action_type': 'move_base',
                'parameter_mapping': {
                    'direction': 1,
                    'distance': 2,
                    'unit': 3
                }
            },
            {
                'pattern': r'(?:turn|rotate|pivot)\s+(left|right|around)\s*(\d*\.?\d*)\s*(degree|degrees|deg)?',
                'intent': 'TURN_DIRECTION_ANGLE',
                'action_type': 'rotate_base',
                'parameter_mapping': {
                    'direction': 1,
                    'angle': 2,
                    'unit': 3
                }
            },
            {
                'pattern': r'(?:move|go|navigate)\s+(?:to|toward|towards)\s+(?:the\s+)?(\w+(?:\s+\w+)*)',
                'intent': 'NAVIGATE_TO_LOCATION',
                'action_type': 'navigate_to_pose',
                'parameter_mapping': {
                    'location': 1
                }
            },
            # Manipulation patterns
            {
                'pattern': r'(?:pick up|grasp|grab|take|lift|collect|get|fetch)\s+(?:the\s+)?(\w+(?:\s+\w+)*)',
                'intent': 'PICK_UP_OBJECT',
                'action_type': 'pick_object',
                'parameter_mapping': {
                    'object': 1
                }
            },
            {
                'pattern': r'(?:place|put|set|drop|release|place down)\s+(?:the\s+)?(\w+(?:\s+\w+)*)\s+(?:on|at|in|to)\s+(?:the\s+)?(\w+(?:\s+\w+)*)',
                'intent': 'PLACE_OBJECT_AT_LOCATION',
                'action_type': 'place_object',
                'parameter_mapping': {
                    'object': 1,
                    'location': 2
                }
            },
            # Transport patterns
            {
                'pattern': r'(?:bring|carry|transport|move|give me|get me)\s+(?:the\s+)?(\w+(?:\s+\w+)*)\s+(?:to|from)\s+(?:me|the\s+)?(\w+(?:\s+\w+)*)?',
                'intent': 'TRANSPORT_OBJECT',
                'action_type': 'transport_object',
                'parameter_mapping': {
                    'object': 1,
                    'destination': 2
                }
            },
            # Communication patterns
            {
                'pattern': r'(?:speak|say|tell|announce|repeat)\s+"([^"]+)"',
                'intent': 'SPEAK_TEXT',
                'action_type': 'speak',
                'parameter_mapping': {
                    'text': 1
                }
            },
            {
                'pattern': r'(?:stop|halt|pause|freeze|wait|hold)\s*(\d*\.?\d*)\s*(second|seconds|minute|minutes)?',
                'intent': 'STOP_OR_WAIT',
                'action_type': 'stop_robot',
                'parameter_mapping': {
                    'duration': 1,
                    'unit': 2
                }
            }
        ]
        
        # Synonym mapping for normalization
        self.synonyms = {
            'forward': ['ahead', 'onward', 'straight'],
            'backward': ['back', 'reverse', 'retreat'],
            'left': ['port', 'larboard'],
            'right': ['starboard'],
            'pick_up': ['grasp', 'grab', 'take', 'lift', 'collect', 'get', 'fetch'],
            'place': ['put', 'set', 'drop', 'release', 'place_down'],
            'navigate': ['go', 'move', 'travel', 'drive', 'walk'],
            'speak': ['say', 'tell', 'announce', 'repeat'],
            'stop': ['halt', 'pause', 'freeze', 'wait', 'hold']
        }
        
        # Normalize command patterns to handle synonyms
        self.normalized_patterns = self.normalize_command_patterns()
        
        self.get_logger().info('Language-Action Mapper initialized')

    def normalize_command_patterns(self):
        """Normalize command patterns to handle synonyms"""
        normalized = []
        
        for pattern_config in self.command_patterns:
            # Create a copy of the pattern config
            norm_config = pattern_config.copy()
            
            # For each synonym set, expand the pattern if needed
            # This is a simplified approach - in practice, you might want more sophisticated normalization
            normalized.append(norm_config)
        
        return normalized

    def command_callback(self, msg):
        """Process natural language command and map to action"""
        command_text = msg.data.lower().strip()
        self.get_logger().info(f'Processing command: {command_text}')
        
        # Normalize the command text
        normalized_command = self.normalize_command(command_text)
        
        # Match command to pattern
        intent_result = self.match_command_to_pattern(normalized_command)
        
        if intent_result:
            intent_type, parameters = intent_result
            self.get_logger().info(f'Matched intent: {intent_type} with parameters: {parameters}')
            
            # Create action from intent
            action = self.create_action_from_intent(intent_type, parameters)
            
            if action:
                # Publish the action
                action_msg = String()
                action_msg.data = json.dumps(action)
                self.action_publisher.publish(action_msg)
                
                self.get_logger().info(f'Published action: {action}')
            else:
                self.get_logger().warn(f'Could not create action for intent: {intent_type}')
        else:
            self.get_logger().warn(f'Could not match command pattern: {command_text}')
            
            # Request clarification
            clarification_msg = String()
            clarification_msg.data = f"Sorry, I didn't understand: '{command_text}'. Could you rephrase that?"
            self.clarification_publisher.publish(clarification_msg)

    def normalize_command(self, command):
        """Normalize command text to handle synonyms and variations"""
        normalized = command
        
        # Replace synonyms with standard terms
        for standard_term, synonyms in self.synonyms.items():
            for synonym in synonyms:
                # Use word boundaries to avoid partial matches
                normalized = re.sub(r'\b' + re.escape(synonym) + r'\b', standard_term, normalized)
        
        return normalized

    def match_command_to_pattern(self, command):
        """Match command to defined patterns and extract parameters"""
        for pattern_config in self.normalized_patterns:
            match = re.search(pattern_config['pattern'], command, re.IGNORECASE)
            if match:
                # Extract parameters based on mapping
                parameters = {}
                
                for param_name, group_idx in pattern_config['parameter_mapping'].items():
                    if group_idx <= len(match.groups()):
                        param_value = match.group(group_idx)
                        if param_value:  # Only add if value exists
                            parameters[param_name] = param_value
                
                # Add the intent type to parameters
                parameters['intent_type'] = pattern_config['intent']
                
                return pattern_config['intent'], parameters
        
        return None

    def create_action_from_intent(self, intent_type, parameters):
        """Create robot action from intent and parameters"""
        action_map = {
            'MOVE_DIRECTION_DISTANCE': self.create_move_direction_action,
            'TURN_DIRECTION_ANGLE': self.create_turn_direction_action,
            'NAVIGATE_TO_LOCATION': self.create_navigate_to_location_action,
            'PICK_UP_OBJECT': self.create_pick_up_object_action,
            'PLACE_OBJECT_AT_LOCATION': self.create_place_object_action,
            'TRANSPORT_OBJECT': self.create_transport_object_action,
            'SPEAK_TEXT': self.create_speak_text_action,
            'STOP_OR_WAIT': self.create_stop_or_wait_action
        }
        
        if intent_type in action_map:
            return action_map[intent_type](parameters)
        else:
            self.get_logger().error(f'Unknown intent type: {intent_type}')
            return None

    def create_move_direction_action(self, params):
        """Create move action from parameters"""
        direction = params.get('direction', 'forward')
        distance_str = params.get('distance', '1')
        unit = params.get('unit', 'meter')
        
        # Convert distance to meters
        distance = float(distance_str)
        if unit and unit.startswith('c'):  # centimeter
            distance = distance / 100.0
        elif unit and unit.startswith('f'):  # foot/feet
            distance = distance * 0.3048  # Convert feet to meters
        
        # Map direction to velocities
        linear_vel = 0.0
        angular_vel = 0.0
        
        if direction in ['forward', 'ahead']:
            linear_vel = 0.5  # m/s
        elif direction in ['backward', 'back']:
            linear_vel = -0.5  # m/s
        elif direction in ['left', 'leftward']:
            angular_vel = 0.5  # rad/s
        elif direction in ['right', 'rightward']:
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
                'duration': duration,
                'distance': distance
            },
            'description': f'Move {direction} by {distance} meters',
            'preconditions': ['robot_is_stationary', 'path_is_clear'],
            'postconditions': ['robot_moved_to_new_position']
        }

    def create_turn_direction_action(self, params):
        """Create turn action from parameters"""
        direction = params.get('direction', 'left')
        angle_str = params.get('angle', '90')
        unit = params.get('unit', 'degree')
        
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

    def create_navigate_to_location_action(self, params):
        """Create navigation action from parameters"""
        location = params.get('location', 'destination')
        
        # In a real implementation, this would look up the location in a map
        # For this example, we'll use a placeholder
        return {
            'action_type': 'navigate_to_pose',
            'parameters': {
                'location_name': location,
                'target_pose': {'x': 1.0, 'y': 1.0, 'theta': 0.0}  # Placeholder
            },
            'description': f'Navigate to {location}',
            'preconditions': ['location_known', 'path_not_blocked'],
            'postconditions': ['robot_at_destination']
        }

    def create_pick_up_object_action(self, params):
        """Create pick up action from parameters"""
        object_name = params.get('object', 'object')
        
        return {
            'action_type': 'pick_object',
            'parameters': {
                'object_name': object_name
            },
            'description': f'Pick up {object_name}',
            'preconditions': ['object_detected', 'object_reachable', 'gripper_free'],
            'postconditions': ['object_grasped', 'gripper_occupied']
        }

    def create_place_object_action(self, params):
        """Create place action from parameters"""
        object_name = params.get('object', 'object')
        location = params.get('location', 'surface')
        
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

    def create_transport_object_action(self, params):
        """Create transport action from parameters"""
        object_name = params.get('object', 'object')
        destination = params.get('destination', 'destination')
        
        # This would be a composite action with multiple steps
        return {
            'action_type': 'composite_action',
            'sub_actions': [
                {
                    'action_type': 'locate_object',
                    'parameters': {'object_name': object_name},
                    'description': f'Locate {object_name}'
                },
                {
                    'action_type': 'pick_object',
                    'parameters': {'object_name': object_name},
                    'description': f'Pick up {object_name}'
                },
                {
                    'action_type': 'navigate_to_pose',
                    'parameters': {'location_name': destination},
                    'description': f'Navigate to {destination}'
                },
                {
                    'action_type': 'place_object',
                    'parameters': {'object_name': object_name, 'location': destination},
                    'description': f'Place {object_name} at {destination}'
                }
            ],
            'description': f'Transport {object_name} to {destination}',
            'preconditions': ['object_detectable', 'destination_known'],
            'postconditions': ['object_at_destination']
        }

    def create_speak_text_action(self, params):
        """Create speak action from parameters"""
        text = params.get('text', 'Hello')
        
        return {
            'action_type': 'speak',
            'parameters': {
                'text': text
            },
            'description': f'Speak: {text}',
            'preconditions': ['speech_system_ready'],
            'postconditions': ['text_spoken']
        }

    def create_stop_or_wait_action(self, params):
        """Create stop/wait action from parameters"""
        duration_str = params.get('duration', '0')
        unit = params.get('unit', 'second')
        
        duration = float(duration_str)
        if unit in ['minute', 'minutes']:
            duration = duration * 60.0  # Convert minutes to seconds
        
        action_type = 'wait' if duration > 0 else 'stop'
        description = f'Wait for {duration} seconds' if duration > 0 else 'Stop current motion'
        
        return {
            'action_type': action_type,
            'parameters': {
                'duration': duration
            },
            'description': description,
            'preconditions': [],
            'postconditions': ['motion_stopped']
        }

def main(args=None):
    rclpy.init(args=args)
    mapper = LanguageActionMapper()
    
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        mapper.get_logger().info('Shutting down language-action mapper...')
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: Computer Vision for Object Identification

### Objective
Implement a computer vision system that identifies objects in the environment and provides information for the VLA system.

### Requirements
- Create object detection node using Isaac ROS packages
- Integrate with ROS 2 messaging system
- Provide 3D pose estimation for detected objects
- Connect with the language-action mapping system
- Implement object tracking for dynamic scenes

### Steps to Complete
1. Create object detection node
2. Implement 3D pose estimation from 2D detection and depth
3. Integrate with ROS 2 topics
4. Connect to the language-action system
5. Add object tracking capabilities
6. Test with various objects and scenarios

### Solution Approach

```python
# object_identification.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class IsaacROSObjectIdentifier(Node):
    def __init__(self):
        super().__init__('object_identifier')
        
        # Subscriptions
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
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10)
        
        # Publishers
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10)
        
        self.pose_publisher = self.create_publisher(
            PoseArray,
            '/object_poses',
            10)
        
        self.tracked_objects_publisher = self.create_publisher(
            Detection2DArray,
            '/tracked_objects',
            10)
        
        # Initialize components
        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_matrix = None
        self.latest_depth = None
        
        # Object tracking
        self.tracked_objects = {}
        self.next_id = 0
        
        # Initialize YOLO model
        try:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None
        
        self.confidence_threshold = 0.5
        
        self.get_logger().info('Isaac ROS Object Identifier initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsics for 3D reconstruction"""
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')

    def image_callback(self, msg):
        """Process image and detect objects"""
        if not self.model or self.camera_matrix is None:
            return
            
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run object detection
            results = self.model(cv_image, conf=self.confidence_threshold)
            
            # Convert results to ROS format
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            # Process each detection result
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
                        class_name = self.model.names[class_id] if class_id < len(self.model.names) else f"unknown_{class_id}"
                        
                        hypothesis.id = class_name
                        hypothesis.score = float(detection.conf[0])
                        
                        detection_msg.results.append(hypothesis)
                        detections_msg.detections.append(detection_msg)
            
            # Publish 2D detections
            self.detection_publisher.publish(detections_msg)
            
            # Estimate 3D poses and publish
            self.estimate_and_publish_poses(detections_msg, msg)
            
            self.get_logger().info(f'Detected {len(detections_msg.detections)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def estimate_and_publish_poses(self, detections_msg, image_msg):
        """Estimate 3D poses from 2D detections and depth information"""
        if self.latest_depth is None:
            self.get_logger().warn('No depth information available for 3D pose estimation')
            return
        
        pose_array = PoseArray()
        pose_array.header = image_msg.header
        
        for detection in detections_msg.detections:
            # Get center of bounding box
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)
            
            # Get depth at center of bounding box
            if (center_y < self.latest_depth.shape[0] and 
                center_x < self.latest_depth.shape[1]):
                
                depth_value = self.latest_depth[center_y, center_x]
                
                if np.isfinite(depth_value) and depth_value > 0:
                    # Convert 2D pixel coordinates to 3D world coordinates
                    fx = self.camera_matrix[0, 0]
                    fy = self.camera_matrix[1, 1]
                    cx = self.camera_matrix[0, 2]
                    cy = self.camera_matrix[1, 2]
                    
                    # Calculate 3D position
                    world_x = (center_x - cx) * depth_value / fx
                    world_y = (center_y - cy) * depth_value / fy
                    world_z = depth_value
                    
                    # Create pose
                    pose = Pose()
                    pose.position.x = world_x
                    pose.position.y = world_y
                    pose.position.z = world_z
                    pose.orientation.w = 1.0  # Identity quaternion
                    
                    pose_array.poses.append(pose)
                    
                    # Add object to tracking
                    self.track_object(detection.results[0].id, pose)
        
        # Publish estimated poses
        self.pose_publisher.publish(pose_array)

    def track_object(self, object_name, pose):
        """Track object across frames"""
        # In a simple implementation, just store the latest pose
        # In a more complex implementation, use tracking algorithms
        self.tracked_objects[object_name] = {
            'pose': pose,
            'last_seen': self.get_clock().now(),
            'id': self.next_id
        }
        self.next_id += 1

def main(args=None):
    rclpy.init(args=args)
    identifier = IsaacROSObjectIdentifier()
    
    try:
        rclpy.spin(identifier)
    except KeyboardInterrupt:
        identifier.get_logger().info('Shutting down object identifier...')
    finally:
        identifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 5: Integration Challenge

### Objective
Integrate all VLA components (Voice processing, Cognitive planning, Language-to-action mapping, Computer vision) into a cohesive system.

### Requirements
- Connect voice processing to cognitive planning
- Link cognitive planning to language-action mapping
- Integrate computer vision for object identification
- Create a unified VLA system
- Implement safety validation for all actions
- Test complete end-to-end functionality

### Solution Approach
1. Create a VLA coordinator node that orchestrates all components
2. Implement message passing between components
3. Add safety validation layer
4. Create end-to-end testing scenarios
5. Validate system performance and safety

### Example Integration Node

```python
# vla_coordinator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray
import json

class VLACoordinator(Node):
    def __init__(self):
        super().__init__('vla_coordinator')
        
        # Subscriptions for all VLA components
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10)
        
        self.interpreted_command_subscription = self.create_subscription(
            String,
            '/interpreted_commands',
            self.interpreted_command_callback,
            10)
        
        self.action_sequence_subscription = self.create_subscription(
            String,
            '/action_sequences',
            self.action_sequence_callback,
            10)
        
        self.object_detection_subscription = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.object_detection_callback,
            10)
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.joint_cmd_publisher = self.create_publisher(
            JointState,
            '/joint_commands',
            10)
        
        self.status_publisher = self.create_publisher(
            String,
            '/vla_status',
            10)
        
        # Internal state
        self.current_objects = {}
        self.active_action_sequence = []
        self.current_action_index = 0
        
        self.get_logger().info('VLA Coordinator initialized')

    def voice_command_callback(self, msg):
        """Process voice command and initiate processing pipeline"""
        command_data = json.loads(msg.data)
        
        # Publish to interpretation system
        interpretation_msg = String()
        interpretation_msg.data = json.dumps({
            'raw_command': command_data,
            'source': 'voice'
        })
        
        # In a real implementation, this would be published to an interpretation topic
        # For this example, we'll simulate the interpretation process
        self.process_interpreted_command(interpretation_msg)

    def interpreted_command_callback(self, msg):
        """Process interpreted command and generate action sequence"""
        interpreted_data = json.loads(msg.data)
        
        # For this example, we'll simulate the action sequence generation
        command = interpreted_data.get('raw_command', {}).get('command', '')
        
        # Generate action sequence based on command
        action_sequence = self.generate_action_sequence(command)
        
        if action_sequence:
            # Publish action sequence
            sequence_msg = String()
            sequence_msg.data = json.dumps({
                'command': command,
                'action_sequence': action_sequence,
                'timestamp': self.get_clock().now().nanoseconds
            })
            
            # In a real implementation, this would be published to an action sequence topic
            # For this example, we'll directly process the sequence
            self.process_action_sequence(sequence_msg)

    def action_sequence_callback(self, msg):
        """Process action sequence and execute actions"""
        sequence_data = json.loads(msg.data)
        action_sequence = sequence_data.get('action_sequence', [])
        
        self.execute_action_sequence(action_sequence)

    def object_detection_callback(self, msg):
        """Update knowledge of visible objects"""
        for detection in msg.detections:
            if detection.results:
                object_name = detection.results[0].id
                confidence = detection.results[0].score
                
                # Update object knowledge if confidence is high enough
                if confidence > 0.7:
                    self.current_objects[object_name] = {
                        'confidence': confidence,
                        'bbox': detection.bbox,
                        'timestamp': self.get_clock().now().nanoseconds
                    }

    def generate_action_sequence(self, command):
        """Generate action sequence from interpreted command"""
        # This is a simplified example - in reality, this would connect to the cognitive planner
        if "move forward" in command:
            return [
                {
                    "action_type": "move_base",
                    "parameters": {"linear_velocity": 0.5, "angular_velocity": 0.0, "duration": 2.0},
                    "description": "Move forward for 2 seconds"
                }
            ]
        elif "pick up" in command:
            object_name = self.extract_object_name(command)
            if object_name in self.current_objects:
                return [
                    {
                        "action_type": "navigate_to_object",
                        "parameters": {"object_name": object_name},
                        "description": f"Navigate to {object_name}"
                    },
                    {
                        "action_type": "pick_object",
                        "parameters": {"object_name": object_name},
                        "description": f"Pick up {object_name}"
                    }
                ]
            else:
                return [
                    {
                        "action_type": "speak",
                        "parameters": {"text": f"Sorry, I cannot see the {object_name}"},
                        "description": f"Speak that {object_name} not found"
                    }
                ]
        else:
            return [
                {
                    "action_type": "speak",
                    "parameters": {"text": f"Sorry, I don't know how to {command}"},
                    "description": f"Speak unrecognized command response"
                }
            ]

    def execute_action_sequence(self, action_sequence):
        """Execute a sequence of actions"""
        self.active_action_sequence = action_sequence
        self.current_action_index = 0
        
        # Execute first action
        if self.active_action_sequence:
            self.execute_current_action()

    def execute_current_action(self):
        """Execute the current action in the sequence"""
        if self.current_action_index >= len(self.active_action_sequence):
            # Sequence complete
            status_msg = String()
            status_msg.data = "ACTION_SEQUENCE_COMPLETE"
            self.status_publisher.publish(status_msg)
            return
        
        action = self.active_action_sequence[self.current_action_index]
        
        # Validate action before execution
        if self.validate_action(action):
            # Execute action based on type
            self.perform_action(action)
            
            # Update status
            status_msg = String()
            status_msg.data = f"EXECUTING_ACTION_{self.current_action_index + 1}_OF_{len(self.active_action_sequence)}"
            self.status_publisher.publish(status_msg)
            
            # Move to next action after delay (in real implementation, this would be event-driven)
            self.current_action_index += 1
            self.get_logger().info(f'Completed action {action["description"]}, moving to next')
            
            # Schedule next action after a delay
            timer = self.create_timer(2.0, self.execute_current_action)
        else:
            self.get_logger().error(f'Action validation failed: {action}')
            # Handle validation failure

    def validate_action(self, action):
        """Validate action for safety and feasibility"""
        action_type = action.get('action_type')
        parameters = action.get('parameters', {})
        
        # Check if action type is supported
        supported_actions = [
            'move_base', 'rotate_base', 'navigate_to_pose', 'pick_object', 
            'place_object', 'speak', 'stop_robot', 'transport_object'
        ]
        
        if action_type not in supported_actions:
            self.get_logger().error(f'Unsupported action type: {action_type}')
            return False
        
        # Validate parameters based on action type
        if action_type in ['move_base', 'rotate_base']:
            required_params = ['linear_velocity', 'angular_velocity', 'duration']
            for param in required_params:
                if param not in parameters:
                    self.get_logger().error(f'Missing required parameter {param} for {action_type}')
                    return False
        
        elif action_type in ['pick_object', 'place_object']:
            if 'object_name' not in parameters:
                self.get_logger().error('Missing required parameter object_name for manipulation action')
                return False
        
        # Safety validation
        if action_type == 'move_base':
            linear_vel = parameters.get('linear_velocity', 0.0)
            if abs(linear_vel) > 1.0:  # Max 1 m/s for safety
                self.get_logger().error(f'Dangerous linear velocity: {linear_vel}')
                return False
        
        return True

    def perform_action(self, action):
        """Perform the specified action"""
        action_type = action['action_type']
        
        if action_type == 'move_base':
            cmd = Twist()
            cmd.linear.x = action['parameters']['linear_velocity']
            cmd.angular.z = action['parameters']['angular_velocity']
            
            # Publish command
            self.cmd_vel_publisher.publish(cmd)
            
        elif action_type == 'speak':
            # In a real implementation, this would interface with a text-to-speech system
            text = action['parameters'].get('text', 'Hello')
            self.get_logger().info(f'Robot would speak: {text}')

    def extract_object_name(self, command):
        """Extract object name from command (simplified)"""
        # This is a simplified extraction - in reality, use NLP techniques
        words = command.lower().split()
        for i, word in enumerate(words):
            if word in ['the', 'a', 'an']:
                if i+1 < len(words):
                    return words[i+1]
        return "object"

def main(args=None):
    rclpy.init(args=args)
    coordinator = VLACoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().info('Shutting down VLA coordinator...')
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Self-Assessment Questions

1. How do you validate that a voice command has been correctly interpreted by the cognitive planner?
2. What are the key differences between simulating sensors in Gazebo vs. Isaac Sim?
3. How would you handle a command that requires an object that isn't currently visible?
4. What safety measures would you implement before executing a manipulation action?
5. How would you verify that the Urdu translation preserves technical accuracy?

## Advanced Challenges (Optional)

1. **Multi-modal Fusion**: Integrate multiple sensor modalities (vision, audio, tactile) for more robust perception
2. **Context-Aware Planning**: Implement context awareness to adapt behavior based on environment and situation
3. **Learning from Interaction**: Design a system that learns from successful and failed interactions to improve future performance
4. **Human-Robot Collaboration**: Implement features that allow humans and robots to collaborate on tasks
5. **Adaptive Personalization**: Create a system that learns user preferences over time and adapts accordingly

## Summary

These exercises provide comprehensive hands-on practice with Vision-Language-Action systems:
- Implementing voice processing and command interpretation
- Creating cognitive planning systems with LLMs
- Developing language-to-action mapping
- Building computer vision systems for object identification
- Integrating all components into a cohesive VLA system

Completing these exercises will strengthen your understanding of VLA systems for humanoid robotics and prepare you for implementing the capstone project.