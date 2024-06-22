import pybullet as p
import pybullet_data
import time
import os
import numpy as np
import cv2

# Connect to PyBullet
p.connect(p.GUI)

# Set the search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Define the path to the URDF files
urdf_root_path = "/home/soumoroy/catkin_ws/src/surge_work/pybullet_ur5_robotiq/urdf/"

# Load the plane
planeId = p.loadURDF("plane.urdf")

# Load the first UR5 robot
ur5_urdf_path = os.path.join(urdf_root_path, "ur5_robotiq_85.urdf")
ur5Id1 = p.loadURDF(ur5_urdf_path, basePosition=[0, 0, 0])

# Load the second UR5 robot
ur5Id2 = p.loadURDF(ur5_urdf_path, basePosition=[1, 0, 0])

# Load a simple block
blockId = p.loadURDF("cube.urdf", basePosition=[0.5, 0, 0], globalScaling=0.1)

# Set gravity
p.setGravity(0, 0, -9.8)

# Simulation time step
p.setTimeStep(1. / 240.)

# Define the RealSense camera setup function
def setup_camera():
    # Set up camera parameters (assuming a static camera for simplicity)
    camera_params = {
        "width": 640,
        "height": 480,
        "fov": 60,
        "aspect": 640 / 480,
        "near": 0.02,
        "far": 1.0,
        "view_matrix": p.computeViewMatrix(
            cameraEyePosition=[0.5, -0.5, 0.5],
            cameraTargetPosition=[0.5, 0, 0.1],
            cameraUpVector=[0, 0, 1]
        ),
        "projection_matrix": p.computeProjectionMatrixFOV(
            fov=60,
            aspect=640 / 480,
            nearVal=0.02,
            farVal=1.0
        )
    }
    return camera_params

# Capture an image from the simulated RealSense camera
def capture_image(camera_params):
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=camera_params["width"],
        height=camera_params["height"],
        viewMatrix=camera_params["view_matrix"],
        projectionMatrix=camera_params["projection_matrix"]
    )
    return rgbImg, depthImg, segImg

# Setup the camera
camera_params = setup_camera()

# Function to move UR5 using inverse kinematics
def move_ur5_to_position(ur5Id, target_position, target_orientation):
    joint_positions = p.calculateInverseKinematics(ur5Id, 6, target_position, targetOrientation=target_orientation)
    for i in range(6):
        p.setJointMotorControl2(ur5Id, i, p.POSITION_CONTROL, joint_positions[i])

# Move both UR5 robots to the block
def move_robots_to_block():
    # Define the block position and orientation
    block_position = [0.5, 0, 0.1]
    block_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    # Move the first UR5 robot to the block position
    move_ur5_to_position(ur5Id1, block_position, block_orientation)
    
    # Adjust the block position for the second robot to grasp from a different side
    block_position_second = [0.5, 0.1, 0.1]
    
    # Move the second UR5 robot to the adjusted block position
    move_ur5_to_position(ur5Id2, block_position_second, block_orientation)

# Lift the block together
def lift_block_together():
    for _ in range(240):
        # Lift the UR5 end-effectors
        target_position_lift = [0.5, 0, 0.3]
        move_ur5_to_position(ur5Id1, target_position_lift, p.getQuaternionFromEuler([0, 0, 0]))
        
        target_position_lift_second = [0.5, 0.1, 0.3]
        move_ur5_to_position(ur5Id2, target_position_lift_second, p.getQuaternionFromEuler([0, 0, 0]))
        
        p.stepSimulation()
        time.sleep(1. / 240.)

# Run the simulation and perform the collaborative task
for _ in range(240):
    p.stepSimulation()
    time.sleep(1. / 240.)

    # Capture an image from the simulated RealSense camera
    rgbImg, depthImg, segImg = capture_image(camera_params)

# Move both UR5 robots to the block
move_robots_to_block()

# Simulate the collaborative lifting task
lift_block_together()

p.disconnect()
