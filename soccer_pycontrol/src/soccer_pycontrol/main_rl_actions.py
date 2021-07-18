#!/usr/bin/env python3
import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
import action_controller

if __name__ == '__main__':

    rospy.init_node("soccer_control")
    rospy.logwarn("Initializing Soccer Control")
    rospy.sleep(1)

    walker = action_controller()
    rospy.logwarn("Starting RL Control Loop")
    walker.run()