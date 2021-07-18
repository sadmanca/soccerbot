import os
from transformation import Transformation
from soccerbot import Soccerbot
from ramp import Ramp
import pybullet as pb
import pybullet_data
from ball import Ball

class ActionController:
    PYBULLET_STEP = 0.004

    def __init__(self):
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0,
                                      cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9,
                         rollingFriction=0.0)

        self.soccerbot = Soccerbot(Transformation())
        self.ball = Ball()

    def run(self, action: str):
        if action == "kick":
            self.run_kick()
        pass

    def get_kick_state_vector(self):
        # get from self.soccerbot and self.ball
        pass

    def run_kick(self):
        # Written by shahryar
        state_vector = self.get_kick_state_vector()

        pass