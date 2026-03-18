import pybullet as p
import pybullet_data
import numpy as np
import os
import time

def dh_transformation(theta, a, d, alpha):
    c1 = np.cos(theta)
    s1 = np.sin(theta)
    c2 = np.cos(alpha)
    s2 = np.sin(alpha)
    return np.array([
        [c1, -s1*c2, s1*s2, a*c1],
        [s1, c1*c2, -c1*s2, a*s1],
        [0, s2, c2, d],
        [0, 0, 0, 1]
    ])
    
def draw_frame(T, lenght=0.2):
    origin = T[:3, 3]
    x_axis = origin + T[:3, 0] * lenght
    y_axis = origin + T[:3, 1] * lenght
    z_axis = origin + T[:3, 2] * lenght
    
    p.addUserDebugLine(origin, x_axis, [1, 0, 0], 2)
    p.addUserDebugLine(origin, y_axis, [0, 1, 0], 2)
    p.addUserDebugLine(origin, z_axis, [0, 0, 1], 2)
    
if __name__ == "__main__":
    
    physicsClient=p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    robot_id = p.loadURDF("2dof_planar3.urdf", basePosition=[0, 0, 0], useFixedBase=True)
    
    joint_angles= [np.pi/8, np.pi/4]
    
    dh_params = [[1, 0, 0], #joint 1
                 [1, 0, 0]] # joint 2
    
    num_dof = 2
    
    T = np.eye(4)
    T_list = np.zeros((num_dof, 4, 4))
    for i in range(num_dof):
        T_i = dh_transformation(joint_angles[i], *dh_params[i])
        T = T @ T_i
        T_list[i] = T
        
    fk_position_dh = T[:3, 3]
    
    for i in range(num_dof):
        p.resetJointState(robot_id, i, joint_angles[i])
        
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240)
        
    ee_state = p.getLinkState(robot_id, 2, computeForwardKinematics=True)
    fk_position_pybullet = np.array(ee_state[4])
    
    green_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 1])
    red_sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])
    
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=green_sphere, basePosition=fk_position_dh)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=red_sphere, basePosition=fk_position_pybullet)
    
    print(f"DH Computed Position: {fk_position_dh}")
    print(f"Pybullet Computed Position: {fk_position_pybullet}")
    
    while True:
        p.stepSimulation()
        time.sleep(1/240)