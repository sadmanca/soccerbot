#!/usr/bin/env python3
import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
# import walking_controller_ros
import walking_controller
import matplotlib as plt
#plt.use('tkagg')

if __name__ == '__main__':

    # rospy.init_node("soccer_control")
    # rospy.logwarn("Initializing Soccer Control")
    # rospy.sleep(1)
    # walker = walking_controller_ros.WalkingControllerRos()
    # rospy.logwarn("Starting Classical Control Loop")
    # walker.run()
    import test
    walker = walking_controller.WalkingController()
    test.WalkingTest.test_walk_1()
