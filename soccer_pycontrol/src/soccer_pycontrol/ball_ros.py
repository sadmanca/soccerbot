import os
from ball import Ball
if os.getenv('ENABLE_PYBULLET', True):
    import pybullet as p


class BallRos(Ball):

    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity

    def get_position(self):
        # TODO jonathan
        pass

    def get_velocity(self):
        # TODO jonathan
        pass