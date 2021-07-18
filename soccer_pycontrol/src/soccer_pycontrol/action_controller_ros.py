from action_controller import ActionController


class ActionControllerRos(ActionController):
    PYBULLET_STEP = 0.004

    def __init__(self):
        super(ActionControllerRos, self).__init__()

    def get_kick_state_vector(self):
        # get from ros topics and stuff
        # need ball position # TODO jonathan get from tf tree, also improve speed of ball updates
        pass

    def run_kick(self):
        # Written by jonathan
        pass