import pybullet as p
import pybullet_data
import numpy as np
import time
import math

# Setup simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

robotId = p.loadURDF("robot_arm.urdf", useFixedBase=True)
linkIndex = 3  # end-effector link

# Get movable joints
movable_joints = []
for i in range(p.getNumJoints(robotId)):
    joint_type = p.getJointInfo(robotId, i)[2]
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        movable_joints.append(i)

print("Movable Joint Indices:", movable_joints)

# Set a configuration for the robot
joint_angles = [0, 0, 0] 

# Apply the configuration
for idx, joint_index in enumerate(movable_joints):
    p.resetJointState(robotId, joint_index, joint_angles[idx])

# Wait 
for _ in range(10):
    p.stepSimulation()
    time.sleep(0.01)

# Get joint values
jointStates = p.getJointStates(robotId, movable_joints)
jointPos = [s[0] for s in jointStates]
zeroVec = [0.0] * len(jointPos)

# Get position of the center of mass for the link
com_pos = p.getLinkState(robotId, linkIndex)[2]

# Compute Jacobian
jacobian = p.calculateJacobian(robotId, linkIndex, com_pos, jointPos, zeroVec, zeroVec)
J_linear = np.array(jacobian[0])
J_angular = np.array(jacobian[1])
J_total = np.vstack((J_linear, J_angular))

# Rank of Jacobian
rank = np.linalg.matrix_rank(J_total)
print("Jacobian Matrix:\n", J_total)
print("Jacobian Rank:", rank)
print("Expected full rank:", len(movable_joints))

if rank < len(movable_joints):
    print("Singularity detected")
else:
    print("No singularity")
