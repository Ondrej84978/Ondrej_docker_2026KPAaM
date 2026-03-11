import pybullet as pb
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

def decompose_homogenous_matrix(T):
    translation = T[:3, 3]
    rotation_matrix = T[:3,:3]
    euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz')
    quaternion = pb.getQuaternionFromEuler(euler_angles)
    return translation, quaternion

def create_cube(position, orientation, color):
    collision_shape_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
    visual_shape_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=color)
    return pb.createMultiBody(1, collision_shape_id, visual_shape_id, position, orientation)

if __name__=="__main__":
    pb.connect(pb.GUI)
    pb.setGravity(0, 0, -9.8)
    
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = pb.loadURDF("plane.urdf")
    
    #initial position of cube
    initial_position = [0, 0, 0.1]
    initial_orientation = pb.getQuaternionFromEuler([0,0,0]) # bez rotacie
    
    create_cube(initial_position, initial_orientation, [1, 0, 0, 1])
    T=np.array([
        [np.cos(np.pi/4), -np.sin(np.pi/4), 0, 0.5],
        [np.sin(np.pi/4), np.cos(np.pi/4),  0, 0.3],
        [0,                 0,              1, 0.1],
        [0,                 0,              0,   1]
    ])@np.array([
        [1,       0,                0,          0.5],
        [0, np.cos(np.pi/4),  -np.sin(np.pi/4), 0.3],
        [0, np.sin(np.pi/4),  np.cos(np.pi/4),  0.1],
        [0,       0,                0,            1]
    ])
    
    new_position, new_orientation = decompose_homogenous_matrix(T)
    
    create_cube(new_position, new_orientation, [0, 1, 0, 1])
    
    num_steps = 1000
    axis_lenght = 0.2
    
    for t in range(num_steps):
        pb.addUserDebugLine(new_position, new_position + T[:3, 0] * axis_lenght, [1, 0, 0], lineWidth=3, lifeTime=1.0 )
        pb.addUserDebugLine(new_position, new_position + T[:3, 1] * axis_lenght, [0, 1, 0], lineWidth=3, lifeTime=1.0 )
        pb.addUserDebugLine(new_position, new_position + T[:3, 2] * axis_lenght, [0, 0, 1], lineWidth=3, lifeTime=1.0 )
        pb.stepSimulation()
        time.sleep(1)
    pb.disconnect()
    
    