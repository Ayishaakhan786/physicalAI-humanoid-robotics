# Perception and AI with Isaac Sim

## 14.1 Realistic Perception Data Generation

Isaac Sim excels at generating highly realistic synthetic data, which is invaluable for training robust AI models for robotics. This involves configuring advanced sensor models and leveraging Isaac Sim's rendering capabilities to produce diverse and accurately labeled data.

### 14.1.1 Configuring Isaac Sim Sensors for Synthetic Data

Isaac Sim supports a wide array of customizable sensors, including:
*   **RGB-D Cameras**: Simulate color, depth, and segmentation images.
*   **LiDARs**: Generate point clouds with realistic reflections and occlusions.
*   **IMUs**: Provide inertial measurements (accelerometer, gyroscope).
*   **Ultrasonics**: Simulate distance measurements.

**High-level Lab Task**: Configure Isaac Sim sensors (RGB-D camera, Lidar) to generate realistic synthetic data.
1.  **Load a Scene**: Open a simple scene with objects (e.g., a room with a table).
2.  **Add Sensors**: From `Create -> Isaac -> Sensors`, add a camera and a LiDAR sensor to the scene. Position them appropriately.
3.  **Configure Properties**: Adjust sensor parameters like field of view, resolution, update rate, and noise models.

## 14.2 Basic Computer Vision with Synthetic Data

Once synthetic data is generated, it can be used for various computer vision tasks. The advantage here is having perfect ground truth labels.

### 14.2.1 Object Detection and Segmentation

Isaac Sim's Replicator extension can automatically generate annotations like bounding boxes, segmentation masks, and object poses for synthetic images.

**Example 14.1: Python Script to Collect Synthetic Images and Annotations (`synthetic_data_collector.py`)**

```python
import carb
from omni.isaac.kit import SimulationApp
CONFIG = {"headless": False}
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
import time
import os

# Replicator imports
from omni.isaac.synthetic_utils import SyntheticDataHelper
import omni.replicator.core as rep

class SyntheticDataCollectorApp:
    def __init__(self):
        self._world = World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()

        # Add some objects to the scene
        self.cuboid = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cuboid",
                name="my_cuboid",
                position=np.array([0.5, 0.0, 0.5]),
                scale=np.array([0.2, 0.2, 0.2]),
                color=np.array([1.0, 0.0, 0.0]),
            )
        )
        self.sphere = self._world.scene.add(
            DynamicCuboid( # Using Cuboid as a proxy for Sphere for simplicity in example
                prim_path="/World/Sphere",
                name="my_sphere",
                position=np.array([-0.5, 0.0, 0.5]),
                scale=np.array([0.2, 0.2, 0.2]), # Assume sphere has same scale visually
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        
        self._world.reset()

        # Setup Replicator
        self._sd_helper = SyntheticDataHelper()
        self._camera_prim_path = "/World/Camera" # Assuming a camera is added manually or programmatically

        # Create a camera if it doesn't exist
        if not simulation_app.get_stage().GetPrimAtPath(self._camera_prim_path):
            rep.create.camera(position=(0, 0, 1.5), look_at=(0,0,0), prim_path=self._camera_prim_path)

        # Attach annotators
        render_product = rep.create.render_product(self._camera_prim_path, (1024, 1024))
        self.writer = rep.WriterRegistry.get("BasicWriter")
        self.writer.initialize(output_dir=os.getcwd() + "/_out_synthetic_data",
                               rgb=True, bounding_box_2d_tight=True)
        
        # Define actions for randomization
        with rep.trigger.on_frame():
            with rep.create.prims(paths=[self.cuboid.prim_path, self.sphere.prim_path], def_prim_type=rep.Light):
                rep.modify.pose(
                    position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 1.5)),
                    rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
                )
            rep.core.detach_all_callbacks() # Ensure Replicator doesn't run indefinitely

    def run(self):
        self._world.run()

        # Simulate and collect data
        for i in range(10): # Collect 10 images
            self._world.step(render=True)
            self.writer.write(render_product) # This writes the data
            if simulation_app.is_exiting():
                break

        print(f"Synthetic data collected to {os.getcwd()}/_out_synthetic_data")
        simulation_app.close()

if __name__ == "__main__":
    app = SyntheticDataCollectorApp()
    app.run()
```

## 14.3 Training AI Models with Synthetic Data

Synthetic data from Isaac Sim can directly be used to train computer vision models, especially when real-world data is scarce or challenging to acquire. This approach helps to overcome data annotation bottlenecks and cover diverse scenarios.

**High-level Lab Task**: Train a simple object detection model using the collected synthetic data.
1.  **Data Preparation**: Organize the collected synthetic images and bounding box annotations into a format suitable for your chosen deep learning framework (e.g., COCO, YOLO format).
2.  **Model Training**: Use a pre-trained object detection model (e.g., YOLOv8, Faster R-CNN) and fine-tune it on your synthetic dataset.
3.  **Evaluation**: Evaluate the model's performance on a separate validation set of synthetic data.

### 14.3.1 Deployment for Real-time Inference

Once trained, AI models can be deployed within Isaac Sim to perform real-time inference on simulated sensor streams.

**Example 14.2: Python Script for Running a Pre-trained Object Detection Model (`object_detector_inference.py`)**

```python
import carb
from omni.isaac.kit import SimulationApp
CONFIG = {"headless": False}
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
import time
import torch # Assuming PyTorch is installed

class ObjectDetectorInferenceApp:
    def __init__(self):
        self._world = World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()

        self.cuboid = self._world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cuboid",
                name="target_cuboid",
                position=np.array([0.5, 0.0, 0.5]),
                scale=np.array([0.2, 0.2, 0.2]),
                color=np.array([1.0, 0.0, 0.0]),
            )
        )
        self._world.reset()

        # --- Load a dummy pre-trained model for demonstration ---
        # In a real scenario, you would load your actual trained object detection model
        # For simplicity, this "model" just pretends to detect the cuboid at its known position.
        self.dummy_model = lambda img: [
            {"box": [0.4, 0.4, 0.6, 0.6], "label": "cuboid", "score": 0.95} if np.random.rand() > 0.1 else {}
        ]
        # --- End dummy model ---

        print("Object detector inference app started.")

    def run(self):
        self._world.run()

        for _ in range(300): # Simulate for 3 seconds
            self._world.step(render=True)
            if simulation_app.is_exiting():
                break

            # Simulate capturing an image from a camera sensor
            # In a real Isaac Sim setup, you would get this from a simulated camera
            simulated_image = np.zeros((480, 640, 3), dtype=np.uint8) # Placeholder for image data

            detections = self.dummy_model(simulated_image)
            for det in detections:
                if det: # Check if detection is not empty
                    print(f"Detected: {det['label']} with score {det['score']} at {det['box']}")
                    # Visualize detection in Isaac Sim (e.g., draw bounding box overlay)
                    # This would involve using omni.synthetic_data.visualization APIs
        
        simulation_app.close()

if __name__ == "__main__":
    app = ObjectDetectorInferenceApp()
    app.run()
```

## 14.4 Reinforcement Learning (RL) Workflows in Isaac Sim

Isaac Sim is a prime environment for **Reinforcement Learning (RL)**, allowing robots to learn complex behaviors through trial and error. Its fast physics engine and Python API enable efficient training loops.

**High-level Lab Task**: Implement a basic RL environment in Isaac Sim to train a robot to perform a simple task.
1.  **Define Environment**: Create an `RLTask` (e.g., `omni.isaac.core.tasks.BaseTask`) that specifies the observation space, action space, reward function, and episode termination conditions.
2.  **Integrate with an RL Agent**: Connect your Isaac Sim environment to an RL framework (e.g., Stable Baselines3, RLib).
3.  **Train Agent**: Run the training loop, observing the agent's learning progress.

**Example 14.3: Python Script Defining a Basic RL Environment (`isaac_sim_rl_env.py`)**

```python
import carb
from omni.isaac.kit import SimulationApp
CONFIG = {"headless": False}
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.core.tasks import BaseTask
import numpy as np
import gym # Assuming OpenAI Gym is installed

class FallingSphereRLTask(BaseTask):
    def __init__(self, name="FallingSphereRLTask", offset=None):
        super().__init__(name, offset)

        self.goal_position = np.array([0.0, 0.0, 0.1]) # Target position for the sphere
        self.sphere = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self.sphere = scene.add(
            DynamicSphere(
                prim_path="/World/RLSphere",
                name="falling_sphere_rl",
                position=np.array([0.0, 0.0, 2.0]),
                radius=0.1,
                mass=1.0,
                color=np.array([0.0, 0.0, 1.0]),
            )
        )
        return

    def get_observations(self):
        # Position of the sphere
        sphere_position = self.sphere.get_world_pose()[0]
        return {"sphere_position": sphere_position}

    def get_reward(self):
        # Simple negative distance from goal as reward
        sphere_position = self.sphere.get_world_pose()[0]
        distance = np.linalg.norm(self.goal_position - sphere_position)
        reward = -distance # Closer to goal = higher reward (less negative)
        return reward

    def is_done(self):
        # Episode ends if sphere is close enough to goal or falls too far
        sphere_position = self.sphere.get_world_pose()[0]
        if np.linalg.norm(self.goal_position - sphere_position) < 0.05:
            return True # Reached goal
        if sphere_position[2] < -1.0: # Fell too far
            return True
        return False

    def get_action_space(self):
        # Example: a 3D force to apply to the sphere
        return gym.spaces.Box(low=-100.0, high=100.0, shape=(3,), dtype=np.float32)

    def apply_action(self, actions):
        # Apply the force to the sphere
        self.sphere.apply_at_point_force(force=actions, point=self.sphere.get_world_pose()[0])

class FallingSphereRLEnv(gym.Env):
    def __init__(self):
        self.simulation_app = SimulationApp(CONFIG)
        self.world = World(stage_units_in_meters=1.0)
        self.task = FallingSphereRLTask()
        self.world.add_task(self.task)
        self.world.reset()

        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32) # Sphere position
        self.action_space = self.task.get_action_space()

    def step(self, action):
        self.task.apply_action(action)
        self.world.step(render=True)
        obs = self.task.get_observations()
        reward = self.task.get_reward()
        done = self.task.is_done()
        info = {} # Additional info
        return obs["sphere_position"], reward, done, info

    def reset(self):
        self.world.reset()
        obs = self.task.get_observations()
        return obs["sphere_position"]

    def close(self):
        self.simulation_app.close()

def main():
    env = FallingSphereRLEnv()
    # You would typically integrate this with an RL agent here (e.g., from Stable Baselines3)
    # For demonstration, we just run a few steps
    obs = env.reset()
    for _ in range(100):
        action = env.action_space.sample() # Random action
        obs, reward, done, info = env.step(action)
        if done:
            print(f"Episode finished with reward: {reward}")
            obs = env.reset()
    env.close()

if __name__ == "__main__":
    main()
```

## Exercises and Practice Tasks

1.  **Synthetic Data Collection**:
    *   Create a simple Isaac Sim scene with a custom object (e.g., a simple cuboid, cylinder, or an imported USD model).
    *   Using the Replicator extension, set up a camera and annotators to collect RGB images and 2D bounding box annotations for your object.
    *   Randomize the object's position, rotation, and lighting for each captured image.
    *   Output: A dataset of synthetic images with corresponding JSON annotations.
2.  **Object Detection with Synthetic Data**:
    *   Research a simple object detection model (e.g., a pre-trained YOLO model).
    *   Use the `object_detector_inference.py` example as a template to load this model into Isaac Sim.
    *   Configure it to process images from a simulated camera and visualize the bounding box detections in real-time in the Isaac Sim viewport.
    *   Output: Real-time object detection visualized in Isaac Sim.
3.  **RL Environment Customization**:
    *   Modify the `isaac_sim_rl_env.py` script to define an RL task for a simple mobile robot (e.g., reaching a target, avoiding obstacles).
    *   Define appropriate observation and action spaces for your robot.
    *   Implement a reward function that encourages the desired behavior.
    *   Output: A custom RL environment for your robot in Isaac Sim.
4.  **Domain Randomization**: Research Domain Randomization techniques. Implement a simple domain randomization strategy within an Isaac Sim scene (e.g., randomizing textures, lighting, or object colors) to create more robust synthetic data. Explain why this is beneficial for sim-to-real transfer.
5.  **Isaac Gym Exploration**: Briefly research NVIDIA Isaac Gym. How does it differ from Isaac Sim, and what are its primary use cases, especially for reinforcement learning?
