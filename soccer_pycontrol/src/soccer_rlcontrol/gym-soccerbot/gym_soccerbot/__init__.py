from gym.envs.registration import register

register(
    id='kick-v0',
    entry_point='gym_soccerbot.envs:Kick',
)

register(
    id='walk-omni-v0',
    entry_point='gym_soccerbot.envs:WalkingOmni',
)

register(
    id='walk-forward-norm-v1',
    entry_point='gym_soccerbot.envs:WalkingForwardNormAgn',
)
register(
    id='norm-v0',
    entry_point='gym_soccerbot.envs:NormAgn',
)
import os
from os.path import dirname as up

def getModelPath(renders=False):
  if renders:
    respath = os.path.join(up(up(up(os.path.dirname(__file__)))), "soccer_description/models/soccerbot_stl.urdf")
  else:
    respath = os.path.join(os.path.dirname(__file__), "soccerbot_empty.urdf")
  return respath

def getBallPath(renders=False):
    return os.path.join(os.path.dirname(__file__), "ball.urdf")