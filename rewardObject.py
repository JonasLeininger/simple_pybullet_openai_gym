import numpy as np

import pybullet as p

class RewardObject:

    def __init__(self, object_type='box'):
        self.cube_start_pos = [0, 0, 0]
        self.cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.angle = 0.
        self.radius = 0.1
        self.object = None

        self.object_type = object_type
        self.reset()

    def reset(self):
        self.radius = np.random.randint(50, 80)/100.
        self.z_axis = np.random.randint(40, 80)/100.
        self.load_object()

    def load_object(self):
        if self.object_type == 'box':
            self.object = p.loadURDF("box.urdf",
                                     self.cube_start_pos,
                                     self.cube_start_orientation,
                                     useFixedBase=0)
        else:
            print('Object not known. Please decide between box and sphere')

    def advance_on_orbit(self):
        shift = np.zeros(3)
        x, y = self.orbit(self.angle)
        shift[0] = x
        shift[1] = y
        shift[2] = self.z_axis
        self.move_floating_box(shift)

    def move_floating_box(self, shift):
        self.cube_start_pos = shift
        p.resetBasePositionAndOrientation(self.object,
                              self.cube_start_pos,
                              self.cube_start_orientation
                              )

    def orbit(self, angle):
        x = np.sin(angle) * self.radius
        y = np.cos(angle) * self.radius
        self.angle += .1
        return x, y
