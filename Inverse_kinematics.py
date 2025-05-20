# Inverse Kinematics

import pybullet as p
import time
import numpy as np
import pybullet_data
from scipy.spatial.transform import Rotation as R

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    # ObjId = p.loadURDF("robot_arm.urdf")
    # robotId = p.loadURDF("robot_arm.urdf", useFixedBase=True)
    # You can also specify the position and orientation by
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    robotId = p.loadURDF("robot_arm.urdf",startPos, startOrientation, useFixedBase=True)
    # bodyUniqueId = ObjId

    jointStates = p.getJointStates(robotId, range(p.getNumJoints(robotId)))
    linkIndex = 3

    # Set joint angles manually (in radians)
    desired_joint_angles = [0, 0.5, -0.3, 0.7]  # You need one value per movable joint
    
    # Apply to robot (assuming 3 DOF arm)
    def set_joint_angles(desired_joint_angles):
        for joint_index in range(len(desired_joint_angles)):
            p.setJointMotorControl2(
                bodyUniqueId=robotId,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=desired_joint_angles[joint_index]
            )
  
    def get_homogeneous_transform(robotId, linkIndex):
        linkState = p.getLinkState(robotId, linkIndex, computeForwardKinematics=True)
        pos = linkState[4]
        orn = linkState[5]

        rot_matrix = R.from_quat(orn).as_matrix()
        transform = np.eye(4)
        transform[:3, :3] = rot_matrix
        transform[:3, 3] = pos
        return transform
   
    while True:
        target_position = [-0.07, 0.13, 0.62]  # x, y, z in meters
        target_orientation = p.getQuaternionFromEuler([0, 0, 0])  # yaw, pitch, roll in radians
        set_joint_angles(desired_joint_angles)

# Calculate IK solution

        ik_solution = p.calculateInverseKinematics(robotId, linkIndex, target_position) # , target_orientation)

        print("Inverse Kinematics Joint Angles:")
        for i, angle in enumerate(ik_solution):
            print(f"Joint {i}: {angle:.4f}")

# Compute FK to verify IK solution

        p.stepSimulation()
        linkState = p.getLinkState(robotId, linkIndex, computeForwardKinematics=True)
        pos = linkState[4]
        orn = linkState[5]
        print(f"FK End-effector Position: {pos}, Orientation (quaternion): {orn}")
        time.sleep(0.5)
        