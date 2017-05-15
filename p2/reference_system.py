import math
import numpy as np

class reference_system:
    def __init__(self, reference_pose):
        self.reference_pose = reference_pose

        angle_cos = math.cos(reference_pose[2])
        angle_sin = math.sin(reference_pose[2])

        # build local to world transformation
        t_trans = np.matrix([[1, 0, reference_pose[0]],
                             [0, 1, reference_pose[1]],
                             [0, 0, 1]])
        t_rot = np.matrix([[angle_cos, -angle_sin, 0],
                           [angle_sin, angle_cos, 0],
                           [0, 0, 1]])
        self.t_to_world = np.dot(t_trans, t_rot)

        # build world to local transformation
        t_trans = np.matrix([[1, 0, -reference_pose[0]],
                             [0, 1, -reference_pose[1]],
                             [0, 0, 1]])
        t_rot = np.matrix([[angle_cos, angle_sin, 0],
                           [-angle_sin, angle_cos, 0],
                           [0, 0, 1]])
        self.t_to_local = np.dot(t_rot, t_trans)

    def to_world_coord(self, local_pose):
        local_matrix = np.matrix([[local_pose[0]], [local_pose[1]], [1]])
        world_matrix = np.dot(self.t_to_world, local_matrix)
        return np.array([world_matrix[0, 0], world_matrix[1, 0], local_pose[2]])

    def to_local_coord(self, world_pose):
        world_matrix = np.matrix([[world_pose[0]], [world_pose[1]], [1]])
        local_matrix = np.dot(self.t_to_local, world_matrix)
        return np.array([local_matrix[0, 0], local_matrix[1, 0], world_pose[2]])
