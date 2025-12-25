# Synthetic Data Generation in Isaac Sim

Synthetic data generation is a critical component of modern AI development, particularly for embodied AI systems like humanoid robots. Isaac Sim provides powerful tools for generating labeled training data that can be used to train perception models with realistic physical and visual properties.

## Learning Objectives

By the end of this section, you will be able to:
- Understand the principles of synthetic data generation for robotics
- Configure Isaac Sim for synthetic data generation workflows
- Implement domain randomization techniques to improve model generalization
- Generate various types of labeled data (images, depth, segmentation masks)
- Create diverse and representative datasets for AI model training
- Validate synthetic data quality against real-world distributions
- Design data generation pipelines that optimize for both diversity and quality

## Introduction to Synthetic Data Generation

Synthetic data generation in Isaac Sim refers to the process of creating artificial data that mimics real-world observations. This is particularly valuable for robotics because:

1. **Cost Reduction**: Eliminates the need for expensive data collection campaigns
2. **Safety**: Allows data collection in dangerous or inaccessible environments
3. **Control**: Enables precise control over scene conditions and object properties
4. **Volume**: Can generate massive datasets quickly
5. **Annotation**: Automatic generation of perfect annotations without manual effort

### Why Synthetic Data Matters for Robotics

Robotic systems require diverse data to handle the variability of real-world environments. For humanoid robotics, this includes:
- Different lighting conditions (indoor, outdoor, low-light)
- Various surface materials (carpet, tile, grass, etc.)
- Multiple object textures and appearances
- Diverse poses and configurations of robots and objects
- Different environmental conditions (weather, obstacles, etc.)

## Domain Randomization

Domain randomization is a technique that varies environmental parameters to improve model generalization. Instead of creating photorealistic scenes, domain randomization creates diverse scenes that help models learn to focus on essential features rather than environmental specifics.

### Key Parameters to Randomize

1. **Lighting Conditions**:
   - Position and intensity of lights
   - Color temperature of light sources
   - Shadow properties
   - Ambient lighting levels

2. **Materials and Textures**:
   - Surface roughness and reflectance
   - Color variations within object classes
   - Texture patterns and scales
   - Material properties (metallic, specular, etc.)

3. **Object Properties**:
   - Position and orientation of objects
   - Size variations within object classes
   - Shape variations for non-rigid objects
   - Object states (open/closed doors, etc.)

4. **Camera Parameters**:
   - Position and orientation
   - Field of view
   - Noise characteristics
   - Distortion parameters

### Implementation Example

Here's how to implement domain randomization in Isaac Sim:

```python
# Example: Randomizing lighting conditions in Isaac Sim
import omni
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import carb
import numpy as np

class DomainRandomizer:
    def __init__(self):
        self.light_prims = []
        self.material_prims = []
        self.object_prims = []
        
    def randomize_lights(self):
        """Randomize lighting properties in the scene"""
        for light_path in self.light_prims:
            light_prim = get_prim_at_path(light_path)
            
            # Randomize light intensity (between 1000 and 3000 lumens)
            intensity = np.random.uniform(1000, 3000)
            light_prim.GetAttribute("inputs:intensity").Set(intensity)
            
            # Randomize color temperature (between 3000K and 6500K)
            color_temp = np.random.uniform(3000, 6500)
            # Convert to RGB approximation
            rgb = self.color_temperature_to_rgb(color_temp)
            light_prim.GetAttribute("inputs:color").Set(rgb)
    
    def randomize_materials(self):
        """Randomize material properties in the scene"""
        for mat_path in self.material_prims:
            mat_prim = get_prim_at_path(mat_path)
            
            # Randomize base color with some variation
            base_color_var = np.random.normal(0, 0.1, 3)
            # Ensure values are in [0, 1] range
            base_color = np.clip(base_color_var, 0, 1)
            mat_prim.GetAttribute("inputs:diffuse_color_constant").Set(base_color)
            
            # Randomize metallic and roughness
            metallic = np.random.uniform(0, 1)
            roughness = np.random.uniform(0.1, 0.9)
            mat_prim.GetAttribute("inputs:metallic_constant").Set(metallic)
            mat_prim.GetAttribute("inputs:roughness_constant").Set(roughness)
    
    def randomize_objects(self):
        """Randomize object properties in the scene"""
        for obj_path in self.object_prims:
            obj_prim = get_prim_at_path(obj_path)
            
            # Randomize position within bounds
            pos_x = np.random.uniform(-5, 5)
            pos_y = np.random.uniform(-5, 5)
            pos_z = np.random.uniform(0, 2)
            obj_prim.GetAttribute("xformOp:translate").Set([pos_x, pos_y, pos_z])
            
            # Randomize rotation
            rot_x = np.random.uniform(-np.pi, np.pi)
            rot_y = np.random.uniform(-np.pi, np.pi)
            rot_z = np.random.uniform(-np.pi, np.pi)
            obj_prim.GetAttribute("xformOp:rotateXYZ").Set([rot_x, rot_y, rot_z])
    
    def color_temperature_to_rgb(self, color_temp):
        """Convert color temperature in Kelvin to RGB values"""
        temp = color_temp / 100
        red, green, blue = 0, 0, 0
        
        # Red calculation
        if temp <= 66:
            red = 1.0
        else:
            red = temp - 60
            red = 329.698727446 * (red ** -0.1332047592)
            red = min(1.0, max(0.0, red / 255))
        
        # Green calculation
        if temp <= 66:
            green = temp
            green = 99.4708025861 * np.log(green) - 161.1195681661
        else:
            green = temp - 60
            green = 288.1221695283 * (green ** -0.0755148492)
        green = min(1.0, max(0.0, green / 255))
        
        # Blue calculation
        if temp >= 66:
            blue = 1.0
        elif temp <= 19:
            blue = 0.0
        else:
            blue = temp - 10
            blue = 138.5177312231 * np.log(blue) - 305.0447927307
            blue = min(1.0, max(0.0, blue / 255))
        
        return [red, green, blue]

# Example usage
randomizer = DomainRandomizer()
# Add your light, material, and object paths to the respective lists
# randomizer.randomize_lights()
# randomizer.randomize_materials()
# randomizer.randomize_objects()
```

## Types of Synthetic Data

### 1. RGB Images
Standard color images from camera sensors:

```python
# Isaac Sim provides a Camera Sensor API
from omni.isaac.sensor import Camera

# Create a camera sensor
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([1.0, 1.0, 1.0]),
    frequency=30,
    resolution=(640, 480)
)

# Capture RGB images
rgb_image = camera.get_rgb()
```

### 2. Depth Data
Per-pixel depth information:

```python
# Get depth data from the camera
depth_data = camera.get_depth()

# Process depth data for use in perception algorithms
# Convert to point cloud if needed
```

### 3. Semantic Segmentation
Pixel-level classification of object classes:

```python
# Isaac Sim provides semantic segmentation capabilities
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Get semantic segmentation data
semantic_data = camera.get_semantic_segmentation()

# The data contains class labels for each pixel
# This is useful for training segmentation models
```

### 4. Instance Segmentation
Pixel-level identification of individual objects:

```python
# Get instance segmentation data
instance_data = camera.get_instance_segmentation()

# Each object instance has a unique ID
# Useful for object detection and tracking models
```

### 5. Normal Maps
Surface normal information for each pixel:

```python
# Get normal map data
normal_map = camera.get_normals()

# Useful for geometry understanding and material analysis
```

## Data Generation Pipelines

### Batch Data Generation

For generating large datasets efficiently:

```python
# Example data generation pipeline
import asyncio
from omni.isaac.synthetic_utils import SyntheticDataHelper

class DataGenerationPipeline:
    def __init__(self, output_dir, num_scenes=1000):
        self.output_dir = output_dir
        self.num_scenes = num_scenes
        self.randomizer = DomainRandomizer()
        
    def generate_scene(self, scene_id):
        """Generate a single scene with randomized parameters"""
        # Randomize the scene
        self.randomizer.randomize_lights()
        self.randomizer.randomize_materials()
        self.randomizer.randomize_objects()
        
        # Capture all required data types
        rgb_img = camera.get_rgb()
        depth_img = camera.get_depth()
        seg_mask = camera.get_semantic_segmentation()
        
        # Save data with appropriate annotations
        self.save_data(scene_id, rgb_img, depth_img, seg_mask)
        
    def save_data(self, scene_id, rgb, depth, segmentation):
        """Save generated data to disk with annotations"""
        import cv2
        import json
        
        # Create directory for this scene
        scene_dir = f"{self.output_dir}/scene_{scene_id:06d}"
        os.makedirs(scene_dir, exist_ok=True)
        
        # Save RGB image
        cv2.imwrite(f"{scene_dir}/rgb.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        
        # Save depth data
        np.save(f"{scene_dir}/depth.npy", depth)
        
        # Save segmentation mask
        cv2.imwrite(f"{scene_dir}/segmentation.png", segmentation)
        
        # Save metadata
        metadata = {
            "scene_id": scene_id,
            "timestamp": time.time(),
            "lighting_conditions": self.randomizer.get_current_lighting_config(),
            "objects_present": self.randomizer.get_current_objects_config()
        }
        
        with open(f"{scene_dir}/metadata.json", 'w') as f:
            json.dump(metadata, f)
    
    def run_batch_generation(self):
        """Generate a batch of scenes"""
        for i in range(self.num_scenes):
            self.generate_scene(i)
            print(f"Generated scene {i+1}/{self.num_scenes}")

# Usage
pipeline = DataGenerationPipeline("./synthetic_dataset", num_scenes=1000)
pipeline.run_batch_generation()
```

## Quality Validation

### Synthetic vs. Real Similarity

To validate the quality of synthetic data, we need to ensure it's similar enough to real data for model transfer:

1. **Statistical Similarity**: Compare statistical properties (histograms, distributions)
2. **Feature Similarity**: Compare features extracted by neural networks
3. **Performance Similarity**: Compare model performance on synthetic vs. real data

### Validation Techniques

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

class DataValidator:
    def __init__(self, synthetic_data_path, real_data_path):
        self.synthetic_data_path = synthetic_data_path
        self.real_data_path = real_data_path
    
    def compare_histograms(self, synthetic_img, real_img, bins=256):
        """Compare histograms of synthetic and real images"""
        # Calculate histograms for each channel
        syn_hist_r, _ = np.histogram(synthetic_img[:,:,0], bins=bins, range=(0,255))
        syn_hist_g, _ = np.histogram(synthetic_img[:,:,1], bins=bins, range=(0,255))
        syn_hist_b, _ = np.histogram(synthetic_img[:,:,2], bins=bins, range=(0,255))
        
        real_hist_r, _ = np.histogram(real_img[:,:,0], bins=bins, range=(0,255))
        real_hist_g, _ = np.histogram(real_img[:,:,1], bins=bins, range=(0,255))
        real_hist_b, _ = np.histogram(real_img[:,:,2], bins=bins, range=(0,255))
        
        # Calculate histogram similarities
        similarity_r = stats.pearsonr(syn_hist_r, real_hist_r)[0]
        similarity_g = stats.pearsonr(syn_hist_g, real_hist_g)[0]
        similarity_b = stats.pearsonr(syn_hist_b, real_hist_b)[0]
        
        avg_similarity = (similarity_r + similarity_g + similarity_b) / 3
        return avg_similarity
    
    def validate_dataset(self):
        """Validate the entire dataset"""
        # Compare a sample of synthetic and real images
        syn_files = os.listdir(self.synthetic_data_path)
        real_files = os.listdir(self.real_data_path)
        
        similarities = []
        for i in range(min(100, len(syn_files), len(real_files))):
            syn_img = plt.imread(f"{self.synthetic_data_path}/{syn_files[i]}")
            real_img = plt.imread(f"{self.real_data_path}/{real_files[i]}")
            
            similarity = self.compare_histograms(syn_img, real_img)
            similarities.append(similarity)
        
        avg_similarity = np.mean(similarities)
        print(f"Average histogram similarity: {avg_similarity:.3f}")
        
        return avg_similarity > 0.7  # Threshold for similarity
```

## Performance Optimization

### Efficient Data Generation

1. **Parallel Processing**: Generate multiple scenes simultaneously
2. **GPU Acceleration**: Use GPU for rendering and processing
3. **Batch Operations**: Process multiple images in batches
4. **Memory Management**: Efficiently manage memory for large datasets

### Multi-Scene Generation

```python
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor

def generate_single_scene(args):
    """Function to generate a single scene (for parallel processing)"""
    scene_id, output_dir, randomizer_config = args
    
    # Initialize Isaac Sim in this process
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    
    try:
        # Set up scene with randomizer config
        # Generate data
        # Save to output_dir
        pass
    finally:
        simulation_app.close()
    
    return scene_id

def parallel_scene_generation(output_dir, num_scenes, num_processes=4):
    """Generate scenes in parallel"""
    args_list = [(i, output_dir, get_random_config()) for i in range(num_scenes)]
    
    with ProcessPoolExecutor(max_workers=num_processes) as executor:
        results = list(executor.map(generate_single_scene, args_list))
    
    print(f"Generated {len(results)} scenes in parallel")
```

## Best Practices for Synthetic Data Generation

### 1. Diversity vs. Realism Balance

- Strive for diverse but physically plausible scenarios
- Don't sacrifice physical plausibility for diversity
- Ensure synthetic data covers the range of real-world scenarios
- Include edge cases and rare scenarios in training data

### 2. Annotation Quality

- Leverage Isaac Sim's perfect annotation capabilities
- Include 3D bounding boxes, poses, and other relevant annotations
- Ensure annotations are consistent across all data modalities
- Include uncertainty estimates where appropriate

### 3. Dataset Curation

- Curate datasets to match the intended application
- Include appropriate balance of different object classes
- Consider the long-tail distribution of real-world data
- Validate dataset quality with domain experts

## Troubleshooting Common Issues

### 1. Data Quality Issues

- **Artifacts**: Check rendering settings and material properties
- **Inconsistent lighting**: Ensure lighting is properly randomized
- **Poor segmentation masks**: Verify object labeling and material assignments

### 2. Performance Issues

- **Slow rendering**: Reduce scene complexity or use lower resolution
- **Memory exhaustion**: Process data in smaller batches
- **Long generation times**: Implement parallel generation pipelines

### 3. Transfer Issues

- **Poor real-world performance**: Improve domain randomization or add real data
- **Overfitting to synthetic patterns**: Increase diversity in generation parameters
- **Domain gap**: Implement sim-to-real techniques like domain adaptation

## Summary

Synthetic data generation in Isaac Sim provides a powerful approach to creating training datasets for AI models in robotics. By leveraging domain randomization and the photorealistic rendering capabilities of Isaac Sim, we can create diverse and representative datasets that enable AI models to learn robust behaviors that transfer to real-world scenarios.

The techniques covered in this section form the foundation for training perception systems in the Physical AI & Humanoid Robotics textbook. In the next section, we'll explore how to integrate Isaac Sim with ROS 2 through the Isaac ROS ecosystem.