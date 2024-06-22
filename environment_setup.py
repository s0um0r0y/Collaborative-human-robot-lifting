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
urdf_root_path = os.path.join(os.getcwd(), "models", "ur_description", "urdf")

# Load the plane
planeId = p.loadURDF("plane.urdf")

# Load the UR5 robot
ur5_urdf_path = os.path.join(urdf_root_path, "/home/soumoroy/catkin_ws/src/surge_work/pybullet_ur5_robotiq/urdf/ur5_robotiq_85.urdf")
ur5Id = p.loadURDF(ur5_urdf_path, basePosition=[0, 0, 0])

# Load a simple block
blockId = p.loadURDF("cube.urdf", basePosition=[0.5, 0, 0], globalScaling=0.1)

# Load the humanoid model
humanId = p.loadURDF("/home/soumoroy/catkin_ws/src/surge_work/pybullet_ur5_robotiq/urdf/humanoid.urdf", basePosition=[1, 1, 0],globalScaling=0.1)

# Set gravity
p.setGravity(0, 0, -9.8)

# Simulation time step
p.setTimeStep(1./240.)

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

# Run the simulation and perform the collaborative task
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1. / 240.)

    # Capture an image from the simulated RealSense camera
    rgbImg, depthImg, segImg = capture_image(camera_params)

    # Human and UR5 robot perform the lifting task together
    # This is a simple example where both agents move together
    # In a real scenario, you would have more complex logic for the collaboration

    # Move the UR5 end-effector to grasp the block
    p.setJointMotorControl2(ur5Id, 1, p.POSITION_CONTROL, targetPosition=0.5)
    p.setJointMotorControl2(ur5Id, 2, p.POSITION_CONTROL, targetPosition=-1.0)
    p.setJointMotorControl2(ur5Id, 3, p.POSITION_CONTROL, targetPosition=1.0)
    p.setJointMotorControl2(ur5Id, 4, p.POSITION_CONTROL, targetPosition=-0.5)
    p.setJointMotorControl2(ur5Id, 5, p.POSITION_CONTROL, targetPosition=1.0)
    p.setJointMotorControl2(ur5Id, 6, p.POSITION_CONTROL, targetPosition=0.5)

    # Move the human model to grasp the block (simple approximation)
    p.resetBasePositionAndOrientation(humanId, [1, 0, 0.25], [1, 0, 0, 1])

p.disconnect()