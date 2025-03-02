# Robotic Arm Grasping Simulation
# Author: Vyshnav K

import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import cv2
import os
from datetime import datetime


class RoboticArmSimulation:

    def __init__(self, render_mode=p.GUI, use_sensor=True):
        """
        Initialize the robotic arm simulation environment

        Args:
            render_mode: GUI for visual rendering, DIRECT for headless simulation
            use_sensor: Boolean to enable/disable sensor-based object detection
        """
        self.render_mode = render_mode
        self.use_sensor = use_sensor

        # Initialize PyBullet
        p.connect(self.render_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Set up camera for recording
        self.width, self.height = 1280, 720
        self.aspect = self.width / self.height
        self.near, self.far = 0.02, 2

        # Setup environment
        self._setup_environment()
        self._load_robot()
        self._setup_soft_object()

        # Initialize sensor if enabled
        if self.use_sensor:
            self._setup_camera_sensor()

        # Video recording setup
        self.record_video = True
        if self.record_video:
            self._setup_video_recording()

    def _setup_environment(self):
        """Set up the simulation environment including table and lighting"""
        try:
            print("Loading plane.urdf...")
            p.loadURDF("plane.urdf")
            print("Plane loaded successfully")

            # Try to load table - using PyBullet's built-in table
            try:
                print("Loading table.urdf...")
                self.table_id = p.loadURDF("table/table.urdf",
                                           basePosition=[0.5, 0, 0],
                                           useFixedBase=True)
                print(f"Table loaded successfully, ID: {self.table_id}")
            except Exception as e:
                print(f"Failed to load table: {e}")
                # Create a simple box as table instead
                self.table_id = p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=p.createCollisionShape(
                        p.GEOM_BOX, halfExtents=[0.6, 0.6, 0.025]),
                    baseVisualShapeIndex=p.createVisualShape(
                        p.GEOM_BOX,
                        halfExtents=[0.6, 0.6, 0.025],
                        rgbaColor=[0.8, 0.8, 0.8, 1]),
                    basePosition=[0.5, 0, 0],
                )
                print(f"Created box as table instead, ID: {self.table_id}")

            # Add better lighting (only in GUI mode)
            if self.render_mode == p.GUI:
                print("Configuring debug visualizer...")
                p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
                p.resetDebugVisualizerCamera(
                    cameraDistance=1.2,
                    cameraYaw=30,
                    cameraPitch=-40,
                    cameraTargetPosition=[0.5, 0, 0.5])
                print("Debug visualizer configured")
        except Exception as e:
            print(f"Error in _setup_environment: {e}")
            raise

    def _load_robot(self):
        """Load the robot model - using PyBullet's available URDF models"""
        # Check which URDFs are available
        print(
            f"Available URDFs in PyBullet data path: {pybullet_data.getDataPath()}"
        )

        # Use KUKA arm which is available in PyBullet data
        try:
            print(f"Looking for KUKA model in: {pybullet_data.getDataPath()}")
            self.robot_id = p.loadURDF("kuka_iiwa/model.urdf",
                                       basePosition=[0, 0, 0.02],
                                       useFixedBase=True)
            print(f"Successfully loaded KUKA arm, ID: {self.robot_id}")
        except Exception as e:
            print(f"Failed to load KUKA model: {e}")
            # Fallback to a different robot if KUKA fails
            self.robot_id = p.loadURDF("franka_panda/panda.urdf",
                                       basePosition=[0, 0, 0.02],
                                       useFixedBase=True)
            print(f"Loaded fallback robot model, ID: {self.robot_id}")

        # Use a simple gripper
        self.gripper_width = 0.08
        self.finger_length = 0.1

        # Create a virtual gripper using simple shapes
        self.gripper_base = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02]),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.02, 0.02, 0.02],
                rgbaColor=[0.3, 0.3, 0.3, 1]),
            basePosition=[0, 0, 0],
        )

        # Create left finger
        self.left_finger = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.04]),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.01, 0.01, 0.04],
                rgbaColor=[0.5, 0.5, 0.5, 1]),
            basePosition=[-0.03, 0, 0],
        )

        # Create right finger
        self.right_finger = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.04]),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.01, 0.01, 0.04],
                rgbaColor=[0.5, 0.5, 0.5, 1]),
            basePosition=[0.03, 0, 0],
        )

        # Connect gripper to robot arm
        self.num_joints = p.getNumJoints(self.robot_id)
        self.end_effector_index = self.num_joints - 1

        # Create constraint between robot and gripper
        self.gripper_constraint = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.end_effector_index,
            childBodyUniqueId=self.gripper_base,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )

        # Create constraints between gripper base and fingers
        self.left_finger_constraint = p.createConstraint(
            parentBodyUniqueId=self.gripper_base,
            parentLinkIndex=-1,
            childBodyUniqueId=self.left_finger,
            childLinkIndex=-1,
            jointType=p.JOINT_PRISMATIC,
            jointAxis=[1, 0, 0],
            parentFramePosition=[-0.03, 0, 0],
            childFramePosition=[0, 0, 0],
            childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))

        p.changeConstraint(self.left_finger_constraint,
                           -0.02,
                           0.05,
                           maxForce=50)

        self.right_finger_constraint = p.createConstraint(
            parentBodyUniqueId=self.gripper_base,
            parentLinkIndex=-1,
            childBodyUniqueId=self.right_finger,
            childLinkIndex=-1,
            jointType=p.JOINT_PRISMATIC,
            jointAxis=[-1, 0, 0],
            parentFramePosition=[0.03, 0, 0],
            childFramePosition=[0, 0, 0],
            childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))

        p.changeConstraint(self.right_finger_constraint,
                           -0.02,
                           0.05,
                           maxForce=50)

        # Set initial joint positions
        self.home_joint_positions = [0, 0, 0, math.pi / 2, 0, -math.pi / 4, 0]
        for i in range(self.num_joints):
            if i < len(self.home_joint_positions):
                p.resetJointState(self.robot_id, i,
                                  self.home_joint_positions[i])

        for joint_index in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            print(
                f"Joint index: {joint_info[0]}, Joint name: {joint_info[1].decode('utf-8')}"
            )

        # Print joint indices for gripper fingers
        print("Left finger joints:")
        for joint_index in range(p.getNumJoints(self.left_finger)):
            joint_info = p.getJointInfo(self.left_finger, joint_index)
            print(
                f"Joint index: {joint_info[0]}, Joint name: {joint_info[1].decode('utf-8')}"
            )

        print("Right finger joints:")
        for joint_index in range(p.getNumJoints(self.right_finger)):
            joint_info = p.getJointInfo(self.right_finger, joint_index)
            print(
                f"Joint index: {joint_info[0]}, Joint name: {joint_info[1].decode('utf-8')}"
            )

    def _setup_soft_object(self):
        """Create a soft object as the target for grasping"""
        # Create a cube or sphere with non-smooth surface
        base_pos = [0.6, 0, 0.6]  # Position on the table

        # Create a visual and collision shape for the soft object
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.025,  # 2.5 cm radius
            rgbaColor=[0.8, 0.2, 0.2, 1.0],
            specularColor=[0.5, 0.5, 0.5])

        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.025  # 2.5 cm radius
        )

        # Create a rigid body that will be somewhat soft through parameters
        self.object_id = p.createMultiBody(
            baseMass=0.05,  # Low mass to make it easier to lift
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=base_pos,
            baseOrientation=[0, 0, 0, 1])

        # Make it "softer" by changing dynamics properties
        p.changeDynamics(self.object_id,
                         -1,
                         lateralFriction=0.9,
                         spinningFriction=0.1,
                         rollingFriction=0.1,
                         restitution=0.1,
                         contactDamping=0.1,
                         contactStiffness=1000)

        # Store object position for later use
        self.object_position = base_pos

    def _setup_camera_sensor(self):
        """Set up an RGB-D camera as a sensor for object detection"""
        self.camera_target_position = [0.6, 0, 0.5]  # Center of the table
        self.camera_distance = 1.0

        # Set up camera for perception
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.camera_target_position,
            distance=self.camera_distance,
            yaw=45,
            pitch=-30,
            roll=0,
            upAxisIndex=2)

        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=self.aspect, nearVal=self.near, farVal=self.far)

    def _setup_video_recording(self):
        """Initialize video recording setup"""
        try:
            # Create a directory for videos if it doesn't exist
            os.makedirs("videos", exist_ok=True)

            # Set up video writer
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_file = f"videos/robotic_arm_simulation_{timestamp}.mp4"

            # Try different codec options
            try:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(video_file, fourcc, 30.0,
                                                    (self.width, self.height))
                print(f"Video will be saved to: {video_file}")
            except Exception as e:
                print(f"Failed with mp4v codec: {e}, trying XVID")
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_file = f"videos/robotic_arm_simulation_{timestamp}.avi"
                self.video_writer = cv2.VideoWriter(video_file, fourcc, 30.0,
                                                    (self.width, self.height))
                print(f"Video will be saved to: {video_file}")
        except Exception as e:
            print(f"Video recording setup failed: {e}")
            self.record_video = False
            print("Video recording has been disabled")

    def capture_frame(self):
        """Capture a frame from the simulation for video recording"""
        if self.record_video:
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0.8, 0, 1],
                distance=1,
                yaw=120,
                pitch=-20,
                roll=0,
                upAxisIndex=2)
            projection_matrix = p.computeProjectionMatrixFOV(
                fov=60, aspect=self.aspect, nearVal=self.near, farVal=self.far)
            _, _, rgb, _, _ = p.getCameraImage(
                width=self.width,
                height=self.height,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix)
            img = np.array(rgb, dtype=np.uint8).reshape(
                (self.height, self.width, 4))  # ensure uint8
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

            # Add timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(img, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2)

            self.video_writer.write(img)
            # except Exception as e:
            #     print(f"Error capturing frame: {e}")
            #     # Don't let frame errors crash the whole simulation
            #     pass

    def detect_object_position(self):
        """
        Use sensor data to detect the object's position

        Returns:
            list: [x, y, z] position of the detected object
        """
        if not self.use_sensor:
            # If sensor is disabled, return the known position
            return self.object_position

        # Get RGB-D image from the camera
        _, _, rgb_img, depth_img, _ = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix)

        # Convert depth image to a numpy array
        depth_img = np.reshape(depth_img, (self.height, self.width))

        # Convert RGB image to a numpy array for object detection
        rgb_img = np.reshape(rgb_img, (self.height, self.width, 4))
        rgb_img = rgb_img[:, :, :3]  # Remove alpha channel

        # Convert to HSV for color-based segmentation
        hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)

        # Define color range for the red object
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_img, lower_red, upper_red)

        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_img, lower_red, upper_red)

        # Combine masks for red detection
        mask = mask1 + mask2

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assumed to be the object)
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M["m00"] > 0:
                # Calculate center of the contour
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Get depth at the center point
                depth = depth_img[cy, cx]

                # Convert image coordinates to world coordinates
                x, y, z = self._pixel_to_world(cx, cy, depth)

                print(
                    f"Object detected at world coordinates: [{x:.3f}, {y:.3f}, {z:.3f}]"
                )
                return [x, y, z]

        print("Object not detected by sensor, using default position")
        return self.object_position

    def _pixel_to_world(self, pixel_x, pixel_y, depth):
        """
        Convert pixel coordinates and depth to world coordinates

        Args:
            pixel_x: x-coordinate in the image
            pixel_y: y-coordinate in the image
            depth: depth value at the pixel

        Returns:
            tuple: (x, y, z) world coordinates
        """
        # Convert depth to distance in the camera frame
        z = self.far * self.near / (self.far - (self.far - self.near) * depth)

        # Normalized device coordinates
        ndcX = (2.0 * pixel_x / self.width) - 1.0
        ndcY = 1.0 - (2.0 * pixel_y / self.height)

        # Unproject to world coordinates
        view_matrix_inv = np.linalg.inv(
            np.array(self.view_matrix).reshape(4, 4).T)
        proj_matrix_inv = np.linalg.inv(
            np.array(self.projection_matrix).reshape(4, 4).T)

        clip_coords = np.array([ndcX, ndcY, -1.0, 1.0])
        eye_coords = np.dot(proj_matrix_inv, clip_coords) * np.array(
            [1, 1, -1, 1])
        ray_eye = eye_coords[:3] / eye_coords[3]
        ray_world = np.dot(view_matrix_inv, np.append(ray_eye, 0))[:3]

        # Normalize ray direction
        ray_world = ray_world / np.linalg.norm(ray_world)

        # Camera position in world coordinates
        camera_pos = np.array(self.camera_target_position) - np.array(
            [0, 0, 0]) * self.camera_distance

        # Calculate the 3D point
        point = camera_pos + ray_world * z
        return point.tolist()

    def move_arm_to_position(self, target_position, orientation=None):
        """
        Move the robotic arm to a target position

        Args:
            target_position: [x, y, z] target position
            orientation: [roll, pitch, yaw] or quaternion for end effector orientation
        """
        # Default orientation if none provided
        if orientation is None:
            orientation = p.getQuaternionFromEuler([0, math.pi / 2, 0])
        elif len(orientation) == 3:
            orientation = p.getQuaternionFromEuler(orientation)

        # Use inverse kinematics to get joint positions
        joint_positions = p.calculateInverseKinematics(self.robot_id,
                                                       self.end_effector_index,
                                                       target_position,
                                                       orientation,
                                                       maxNumIterations=100,
                                                       residualThreshold=1e-5)

        # Set joint positions
        for i in range(self.num_joints):
            if i < len(joint_positions):
                p.setJointMotorControl2(bodyUniqueId=self.robot_id,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=joint_positions[i],
                                        force=100)

        # Wait for the arm to reach the position
        self._wait_for_arm_to_settle()

    def _wait_for_arm_to_settle(self, threshold=0.01, max_steps=100):
        """
        Wait for the robotic arm to settle at its target position

        Args:
            threshold: Maximum allowed joint velocity magnitude to consider arm settled
            max_steps: Maximum number of simulation steps to wait
        """
        for _ in range(max_steps):
            still_moving = False

            # Check joint velocities
            for i in range(self.num_joints):
                joint_state = p.getJointState(self.robot_id, i)
                joint_velocity = joint_state[1]  # Index 1 contains velocity

                if abs(joint_velocity) > threshold:
                    still_moving = True
                    break

            if not still_moving:
                break

            # Simulation step
            p.stepSimulation()
            self.capture_frame()
            time.sleep(
                1 / 240.0)  # Slow down simulation for smoother visualization

    def open_gripper(self):
        """Opens the gripper by moving the fingers outward."""
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=
            0,  # Replace with the correct joint index for the left finger
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.05,
            force=50)
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=
            1,  # Replace with the correct joint index for the right finger
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.05,
            force=50)

    def close_gripper(self):
        """Closes the gripper by moving the fingers inward."""
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=
            0,  # Replace with the correct joint index for the left finger
            controlMode=p.POSITION_CONTROL,
            targetPosition=-0.02,
            force=50)
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=
            1,  # Replace with the correct joint index for the right finger
            controlMode=p.POSITION_CONTROL,
            targetPosition=-0.02,
            force=50)
        # Allow time for gripper to close
        for _ in range(40):
            p.stepSimulation()
            self.capture_frame()
            time.sleep(1 / 240.0)

    def run_grasping_task(self):
        """Execute the full grasping task sequence"""
        # Move to home position first
        print("Moving to home position...")
        for i in range(self.num_joints):
            if i < len(self.home_joint_positions):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=self.home_joint_positions[i],
                    force=100)

        # Wait for the arm to reach home position
        self._wait_for_arm_to_settle()

        # Detect object position using sensor if enabled
        print("Detecting object position...")
        object_position = self.detect_object_position()
        print(f"Object detected at: {object_position}")

        # Open gripper
        print("Opening gripper...")
        self.open_gripper()

        # Move above the object
        approach_position = [
            object_position[0], object_position[1], object_position[2] + 0.1
        ]
        print(f"Moving above object at: {approach_position}")
        self.move_arm_to_position(approach_position,
                                  orientation=[0, math.pi / 2, 0])

        # Move to grasping position
        grasp_position = [
            object_position[0], object_position[1], object_position[2] + 0.03
        ]
        print(f"Moving to grasping position: {grasp_position}")
        self.move_arm_to_position(grasp_position,
                                  orientation=[0, math.pi / 2, 0])

        # Close gripper to grasp the object
        print("Closing gripper to grasp object...")
        self.close_gripper()

        # Lift the object
        lift_position = [
            object_position[0], object_position[1], object_position[2] + 0.2
        ]
        print(f"Lifting object to: {lift_position}")
        self.move_arm_to_position(lift_position,
                                  orientation=[0, math.pi / 2, 0])

        # Move to a different position with the object
        new_position = [
            object_position[0] - 0.2, object_position[1] + 0.1,
            object_position[2] + 0.2
        ]
        print(f"Moving object to new position: {new_position}")
        self.move_arm_to_position(new_position,
                                  orientation=[0, math.pi / 2, 0])

        # Place the object
        place_position = [
            new_position[0], new_position[1], object_position[2] + 0.03
        ]
        print(f"Placing object at: {place_position}")
        self.move_arm_to_position(place_position,
                                  orientation=[0, math.pi / 2, 0])

        # Open gripper to release object
        print("Opening gripper to release object...")
        self.open_gripper()

        # Move arm back to home position
        print("Moving back to home position...")
        self.move_arm_to_position([0.3, 0, 0.6],
                                  orientation=[0, math.pi / 2, 0])

        # Return to complete home position
        for i in range(self.num_joints):
            if i < len(self.home_joint_positions):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=self.home_joint_positions[i],
                    force=100)

        # Wait for the arm to reach home position
        self._wait_for_arm_to_settle()

        print("Grasping task completed successfully!")

    def close(self):
        """Clean up resources and close the simulation"""
        if self.record_video:
            self.video_writer.release()
        p.disconnect()


def main():
    try:
        simulation = RoboticArmSimulation(render_mode=p.DIRECT,
                                          use_sensor=True)
        print("Connected to PyBullet in DIRECT (headless) mode")

        # Print PyBullet version and configuration
        print(f"PyBullet version: {p.getAPIVersion()}")
        print(f"PyBullet data path: {pybullet_data.getDataPath()}")

        # Allow time for the physics to settle
        print("Initializing simulation...")
        for i in range(100):
            p.stepSimulation()
            simulation.capture_frame()
            if i % 20 == 0:  # Print status periodically
                print(f"Simulation step {i}/100...")
            time.sleep(1 / 240.0)

        # Run the grasping task
        simulation.run_grasping_task()

        # Keep the simulation running for a bit at the end
        print("Simulation completed, keeping simulation running...")
        for _ in range(200):
            p.stepSimulation()
            simulation.capture_frame()
            time.sleep(1 / 240.0)

        # Close the simulation
        simulation.close()
        print("Simulation closed.")

    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
