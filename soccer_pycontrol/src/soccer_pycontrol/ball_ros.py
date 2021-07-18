import os
from ball import Ball
if os.getenv('ENABLE_PYBULLET', False):
    import pybullet as p


class BallRos(Ball):

    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity
        self.path = 'TODO hardcode'
        self.plane = p.loadURDF(self.path, basePosition=self.position,
                                baseOrientation=[0, 0, 0, 1])

    def get_position(self):
        # TODO jonathan
        pass

    def get_velocity(self):
        # TODO jonathan
        pass