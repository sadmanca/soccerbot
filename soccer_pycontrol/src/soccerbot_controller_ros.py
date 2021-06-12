from soccerbot_controller import *
import rospy
from soccerbot_ros import SoccerbotRos
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty

class SoccerbotControllerRos(SoccerbotController):

    def __init__(self):
        if os.getenv('COMPETITION', 'false') == 'true':
            pb.connect(pb.DIRECT)
        else:
            pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)

        self.soccerbot = SoccerbotRos(Transformation(), useFixedBase=False)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.robot_position_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.robot_pose_callback)
        self.terminate_walk_subscriber = rospy.Subscriber("terminate_walking", Empty, self.terminate_walk_callback)
        self.completed_walk_publisher = rospy.Publisher("completed_walking", Empty, queue_size=1)
        self.goal = PoseStamped()
        self.new_goal = self.goal
        self.terminate_walk = False

    def robot_pose_callback(self, pose):
        self.robot_pose = pose
        pass

    def goal_callback(self, pose):
        self.new_goal = pose
        pass

    def terminate_walk_callback(self, val):
        self.terminate_walk = True

    def pose_to_transformation(self, pose):
        t = Transformation([pose.position.x, pose.position.y, pose.position.z],
                           [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return t

    def transformation_to_pose(self, trans: Transformation) -> PoseStamped:
        t = PoseStamped()
        t.pose.position.x = trans.get_position()[0]
        t.pose.position.y = trans.get_position()[1]
        t.pose.position.z = trans.get_position()[2]

        t.pose.orientation.x = trans.get_orientation()[0]
        t.pose.orientation.y = trans.get_orientation()[1]
        t.pose.orientation.z = trans.get_orientation()[2]
        t.pose.orientation.w = trans.get_orientation()[3]
        return t

    def ready(self):
        pass

    def setGoal(self, goal):
        self.goal_callback(self.transformation_to_pose(goal))

    def wait(self, steps):
        for i in range(steps):
            rospy.sleep(SoccerbotController.PYBULLET_STEP)
            pb.stepSimulation()

    def run(self):
        t = 0
        r = rospy.Rate(1/SoccerbotController.PYBULLET_STEP)

        self.soccerbot.ready()
        self.soccerbot.reset_head()
        self.soccerbot.publishAngles()

        while not rospy.is_shutdown():
            if self.new_goal != self.goal:
                print("Recieved New Goal")
                self.soccerbot.setPose(self.pose_to_transformation(self.robot_pose.pose.pose))
                self.wait(200)

                self.goal = self.new_goal
                self.soccerbot.ready() # TODO Cancel walking
                self.soccerbot.reset_head()
                self.soccerbot.publishAngles()
                print("Getting ready")
                self.wait(150)


                # Reset robot position and goal
                self.soccerbot.setGoal(self.pose_to_transformation(self.goal.pose))
                self.soccerbot.publishPath()
                t = 0

            if self.terminate_walk:
                if self.soccerbot.robot_path != None:
                    self.soccerbot.ready()
                    self.soccerbot.publishAngles()
                    print("Terminating Walk")
                    t = self.soccerbot.robot_path.duration() + 1
                self.terminate_walk = False

            if self.soccerbot.robot_path is not None and self.soccerbot.current_step_time <= t <= self.soccerbot.robot_path.duration():
                self.soccerbot.stepPath(t, verbose=True)
                self.soccerbot.apply_imu_feedback(t, self.soccerbot.get_imu())
                forces = self.soccerbot.apply_foot_pressure_sensor_feedback(self.ramp.plane)
                pb.setJointMotorControlArray(bodyIndex=self.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                             jointIndices=list(range(0, 20, 1)),
                                             targetPositions=self.soccerbot.get_angles(),
                                             forces=forces
                                             )
                self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_size
                self.soccerbot.publishOdometry()

            if self.soccerbot.robot_path is not None and t <= self.soccerbot.robot_path.duration() < t + SoccerbotController.PYBULLET_STEP:
                print("Completed Walk")
                e = Empty()
                self.completed_walk_publisher.publish(e)

            if self.soccerbot.robot_path is None or t > self.soccerbot.robot_path.duration():
                self.soccerbot.apply_head_rotation()


            self.soccerbot.publishAngles()
            pb.stepSimulation()
            t = t + SoccerbotController.PYBULLET_STEP
            r.sleep()
