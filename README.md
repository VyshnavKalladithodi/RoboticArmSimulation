# Robotic Arm Grasping Simulation

This project implements a simulation of a robotic arm grasping a small soft object with a non-smooth surface. The solution includes both basic grasping functionality and sensor-based object detection.

## Project Structure

```
robotic-arm-simulation/
├── main.py        # Main simulation code
├── videos/        # Generated demonstration videos
├── console log.txt # Console log of a simulation run
└── README.md      # This file
```

## Prerequisites

- Python 3.8 or higher
- PyBullet
- NumPy
- OpenCV

## Installation

1. Clone the repository (if applicable):

    ```bash
    git clone <repository_url>
    cd robotic-arm-simulation
    ```

2. Install the required dependencies:

    ```bash
    pip install pybullet numpy opencv-python
    ```

3. Ensure all required asset files (URDFs) are accessible by PyBullet. The code uses the data provided by the `pybullet_data` package, so ensure it is correctly installed.

## Usage

Run the simulation with:

```bash
python main.py
```

By default, the simulation attempts to run in GUI mode first. If GUI mode fails (due to environment limitations), it falls back to DIRECT (headless) mode. The simulation performs the following steps:

1. Initializes the PyBullet physics environment.
2. Loads the plane, table, and KUKA robotic arm.
3. Loads the gripper fingers and sets up constraints.
4. Loads the soft object (sphere or cube).
5. If sensor-based detection is enabled, it detects the object's position.
6. Moves the robotic arm to a home position.
7. Moves the arm to a position above the object.
8. Moves the arm to a grasping position.
9. "Closes" the gripper (functionality modified due to URDF limitations).
10. Lifts the object.
11. Moves the object to a new location.
12. Places the object.
13. "Opens" the gripper (functionality modified due to URDF limitations).
14. Moves back to the home position.
15. Records a video of the simulation.

## Key Features

- **Robotic Arm Control**: Simulates the movement and grasping actions of a KUKA robotic arm.
- **Soft Object Grasping**: Attempts to grasp a soft object (sphere or cube). The gripper functionality has been modified due to URDF issues.
- **Sensor-Based Object Detection**: Includes an optional sensor-based object detection mechanism (RGB-D camera simulation).
- **Video Recording**: Records a video of the simulation, even in headless mode.
- **Headless Mode Support**: Runs in headless mode (`p.DIRECT`) when GUI is not available.

## Code Structure and Analysis

### Class: `RoboticArmSimulation`

#### `__init__(self, render_mode=p.GUI, use_sensor=True)`
- Initializes the simulation environment.
- `render_mode`: Specifies whether to use GUI or DIRECT mode for rendering.
- `use_sensor`: Enables or disables sensor-based object detection.
- Connects to PyBullet and sets up the environment, robot, and object.
- Sets up video recording if `record_video` is enabled.

#### `_setup_environment(self)`
- Loads the plane and table URDFs.
- Handles errors in table loading by creating a simple box.
- Configures debug visualizer settings if in GUI mode.

#### `_load_robot(self)`
- Loads the KUKA robotic arm URDF.
- If loading fails, attempts to load a Franka Panda robot.
- Creates a virtual gripper and constraints.
- Prints loaded joints and sets home positions.

#### `_setup_soft_object(self)`
- Creates a soft object (sphere) with specified properties.

#### `_setup_camera_sensor(self)`
- Configures a virtual RGB-D camera for object detection.

#### `_setup_video_recording(self)`
- Creates a directory for videos.
- Initializes an OpenCV video writer.
- Handles codec errors by trying different options.

#### `capture_frame(self)`
- Captures a frame from PyBullet's camera and overlays a timestamp.
- Writes the frame to the video file.

#### `detect_object_position(self)`
- If `use_sensor` is disabled, returns the default object position.
- Otherwise, captures RGB-D images and detects objects using color segmentation.
- Converts detected pixel coordinates and depth into world coordinates.

#### `_pixel_to_world(self, pixel_x, pixel_y, depth)`
- Converts 2D pixel coordinates and depth into 3D world coordinates using camera parameters.

#### `move_arm_to_position(self, target_position, orientation=None)`
- Moves the robotic arm to a target position using inverse kinematics.

#### `_wait_for_arm_to_settle(self, threshold=0.01, max_steps=100)`
- Waits for the robotic arm to stabilize at the target position.

#### `open_gripper(self)`, `close_gripper(self)`
- Controls the gripper's opening and closing movements.

#### `run_grasping_task(self)`
- Executes the full grasping sequence:
  1. Moves to home position.
  2. Detects the object position.
  3. Opens the gripper.
  4. Moves above and then to the object.
  5. Closes the gripper.
  6. Lifts and moves the object.
  7. Places and releases the object.
  8. Returns to home position.

#### `close(self)`
- Releases video resources and disconnects from PyBullet.

## Object Detection Algorithm

1. **Capture RGB-D Image**: Retrieves images from the camera sensor.
2. **Convert to HSV**: Processes images in the HSV color space.
3. **Color Masking**: Identifies red objects using thresholding.
4. **Morphological Operations**: Cleans up the mask with opening/closing operations.
5. **Contour Detection**: Extracts object contours.
6. **Depth Retrieval**: Obtains depth information for the detected object.
7. **Pixel-to-World Conversion**: Transforms 2D detection into 3D coordinates.

## Robot Movements

- **Inverse Kinematics**: Uses `p.calculateInverseKinematics()` to determine joint angles.
- **Joint Motor Control**: Uses `p.setJointMotorControl2()` to apply movements.
- **Settling Check**: Ensures stability before proceeding with further actions.

## Troubleshooting

- **GUI Issues**: If you encounter GUI issues, try running the simulation in `p.DIRECT` mode.
- **Joint Index Errors**: Verify the joint indices for the gripper fingers.
- **Video Corruption**: Ensure proper video writer closure and check frame data.
- **Object Detection**: If object detection fails, check the camera sensor setup and object placement.
- **Constraint Errors**: If you encounter constraint errors, ensure the correct use of the `changeConstraint` method.

## Author

Vyshnav K

