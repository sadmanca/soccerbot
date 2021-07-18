import os
import rospy
if os.getenv('ENABLE_PYBULLET', False):
    import pybullet as p


class Ball:

    def __init__(self, position=[0,0], velocity=[0,0]):
        self.position = position
        self.velocity = velocity
        self.path = 'TODO hardcode'
        self.plane = p.loadURDF(self.path, basePosition=[0, 0, 0],
                                baseOrientation=[0, 0, 0, 1])

    def set_position(self):
        pass

    def set_velocity(self):
        pass

    def get_position(self):
        pass

    def get_velocity(self):
        pass