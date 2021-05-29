import random

from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator

from robot import Robot
from ball import Ball
from strategy import DummyStrategy
import math
import numpy as np
import copy



class GameEngine:
    PHYSICS_UPDATE_INTERVAL = 0.1
    STRATEGY_UPDATE_INTERVAL = 5  # Every 5 physics steps
    DISPLAY_UPDATE_INTERVAL = 5  # Every 5 physics steps

    def __init__(self):
        # Initialize robots
        self.robots = [
            Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                  position=np.array([0.0, -3.5, math.pi / 2])),
            Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([-1.5, -1.5, math.pi / 2])),
            Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([1.5, -1.5, math.pi / 2])),
            Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.STRIKER, status=Robot.Status.READY,
                  position=np.array([0.0, -0.8, math.pi / 2])),
            Robot(team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                  position=np.array([0.0, 3.5, -math.pi / 2])),
            Robot(team=Robot.Team.OPPONENT, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([-1.5, 1.5, -math.pi / 2])),
            Robot(team=Robot.Team.OPPONENT, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([1.5, 1.5, -math.pi / 2])),
            Robot(team=Robot.Team.OPPONENT, role=Robot.Role.STRIKER, status=Robot.Status.READY,
                  position=np.array([0.0, 0.8, -math.pi / 2]))
        ]
        self.ball = Ball(position=np.array([0, 0]))

        self.robots_init = copy.deepcopy(self.robots)
        self.ball_init = copy.deepcopy(self.ball)

        # Initialize display
        # Rules and Dimensions https://cdn.robocup.org/hl/wp/2021/04/V-HL21_Rules_changesMarked.pdf

        fig = plt.figure(figsize=(6.0, 9.0), dpi=60)
        background = fig.add_axes([0, 0, 1, 1])
        background.axis('equal')
        background.set_xlim([-3.5, 3.5])
        background.set_ylim([-5, 5])
        background.xaxis.set_major_locator(MultipleLocator(1))
        background.yaxis.set_major_locator(MultipleLocator(1))
        background.xaxis.set_minor_locator(MultipleLocator(0.1))
        background.yaxis.set_minor_locator(MultipleLocator(0.1))
        background.grid(which='minor', alpha=0.2)
        background.grid(which='major', alpha=0.5)
        background.add_patch(plt.Rectangle((-3, -4.5), 6, 9, alpha=0.1, color='green'))
        background.add_patch(plt.Rectangle((-1.3, -4.55), 2.6, 0.05, color='blue'))
        background.add_patch(plt.Rectangle((-1.3, 4.5), 2.6, 0.05, color='blue'))
        background.add_line(plt.Line2D((-3, 3), (0, 0), color='blue'))
        background.add_patch(plt.Circle((-0, 0), 13/ 2, fill=None, color='blue'))
        foreground = fig.add_axes([0, 0, 1, 1])
        foreground.set_facecolor((0, 0, 0, 0))

        # Setup the strategy
        self.strategy = DummyStrategy()

    def run(self):
        game_period_steps = int(2 * 2 * 60 / GameEngine.PHYSICS_UPDATE_INTERVAL)  # 2 Periods of 2 minutes each

        friendly_points = 0
        opponent_points = 0

        for step in range(game_period_steps):
            if step == int(game_period_steps / 2):
                print("Second Half Started: ")
                self.resetRobots()

            if step % GameEngine.STRATEGY_UPDATE_INTERVAL == 0:
                self.strategy.update_both_team_strategy(self.robots, self.ball)

            self.updateEstimatedPhysics(self.robots, self.ball)

            # Check victory condition
            if self.ball.get_position()[1] > 4.5:
                print("Friendly Scores!")
                friendly_points += 1
                self.resetRobots()
            elif self.ball.get_position()[1] < -4.5:
                print("Opponent Scores!")
                opponent_points += 1
                self.resetRobots()

            if step % GameEngine.DISPLAY_UPDATE_INTERVAL == 0:
                self.displayGameState(self.robots, self.ball, step * GameEngine.PHYSICS_UPDATE_INTERVAL)

        print(F"Game Finished: Friendly: {friendly_points}, Opponent: {opponent_points}")
        plt.show()

    def displayGameState(self, robots, ball, t=0.0):
        foreground = plt.gcf().axes[1]
        foreground.clear()
        foreground.axis('equal')
        foreground.set_xlim([-3.5, 3.5])
        foreground.set_ylim([-5, 5])

        # Display Robots
        for robot in robots:
            x = robot.get_position()[0]
            y = robot.get_position()[1]
            theta = robot.get_position()[2]

            if robot.team == Robot.Team.OPPONENT:
                color = 'red'
            else:
                color = 'green'

            if robot.status == Robot.Status.FALLEN_BACK:
                color = 'yellow'
            foreground.add_patch(plt.Circle((x, y), 0.08, color=color))

            arrow_len = 0.3
            arrow_end_x = math.cos(theta) * arrow_len
            arrow_end_y = math.sin(theta) * arrow_len
            foreground.arrow(x, y, arrow_end_x, arrow_end_y, head_width=0.05, head_length=0.1, color=color)

        # Draw ball
        x = ball.get_position()[0]
        y = ball.get_position()[1]
        dx = ball.get_velocity()[0]
        dy = ball.get_velocity()[1]
        foreground.add_patch(plt.Circle((x, y), 0.5 / 2 / math.pi, color='black'))
        foreground.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1)

        # GUI text
        foreground.text(-3, 4.5, "Time: {0:.6g}".format(t))

        plt.pause(0.001)

    def updateEstimatedPhysics(self, robots, ball):
        # Robot do action in a random priority order

        for robot in sorted(robots,key=lambda _: random.random()):
            # TODO use the same trajectory as in soccer_pycontrol
            if robot.status == Robot.Status.WALKING:
                delta = (robot.goal_position - robot.get_position())[0:2]
                if np.linalg.norm(delta) == 0:
                    continue

                unit = delta / np.linalg.norm(delta)
                robot.position[0:2] = robot.get_position()[0:2] + unit * robot.speed * GameEngine.PHYSICS_UPDATE_INTERVAL
                for otherRobot in robots:
                    position_equal = (abs(robot.position[0:2].round(2) - otherRobot.position[0:2].round(2)) < [0.16, 0.16]).all()
                    if position_equal and robot != otherRobot:
                        robot.status = Robot.Status.FALLEN_BACK
                        otherRobot.status = Robot.Status.FALLEN_BACK
            # TODO if walk into another robot, stop moving and fallback
            elif robot.status == Robot.Status.FALLEN_BACK or robot.status == Robot.Status.FALLEN_FRONT:
                if robot.fallen_timeout == 0:
                    robot.status = Robot.Status.WALKING
                    robot.fallen_timeout = 10
                elif robot.fallen_timeout > 0:
                    robot.fallen_timeout -= 1
            elif robot.status == Robot.Status.KICKING:
                if ball.kick_timeout == 0:
                    ball.velocity = robot.kick_velocity
                    ball.kick_timeout = 10
                robot.status = Robot.Status.READY

        # Ball
        if ball.kick_timeout > 0:
            ball.kick_timeout = ball.kick_timeout - 1

        # TODO If ball hits a person, bounce
        if (abs(robot.position[0:2].round(2) - ball.position[0:2].round(2)) < [0.08 + 0.25/math.pi, 0.08 + 0.25/math.pi]).all() and robot.status != Robot.Status.KICKING:
            ball.velocity = ball.velocity * -1
            '''
            ball_velo_theta = math.atan(ball.position[1]/ball.position[0])
            ball.velocity[0] = ball.velocity[0]*math.cos(ball_velo_theta)
            ball.velocity[1] = ball.velocity[1]*math.sin(ball_velo_theta)
            '''
            ball.bounce_counter += 1
            print('Bounce! '+str(ball.bounce_counter))

        ball.position = ball.get_position() + ball.get_velocity() * GameEngine.PHYSICS_UPDATE_INTERVAL
        ball.velocity = ball.velocity * Ball.FRICTION_COEFF


    def resetRobots(self):
        self.robots = copy.deepcopy(self.robots_init)
        self.ball = copy.deepcopy(self.ball_init)