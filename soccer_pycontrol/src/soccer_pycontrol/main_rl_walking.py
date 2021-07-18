#!/usr/bin/env python3
import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
import walking_controller_ros_rl

if __name__ == '__main__':

    rospy.init_node("soccer_control_rl")
    rospy.logwarn("Initializing Soccer RL Control")
    rospy.sleep(1)

    walker = walking_controller_ros_rl.WalkingControllerRosRl()
    rospy.logwarn("Starting RL Control Loop")
    walker.run()