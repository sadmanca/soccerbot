#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float64
from sensor_msgs import JointState
import tf


# soccer_webot/scripts/start_single.py for param
# soccer_traject/src//trajectory.py for fillign out jointstate
# next step look at commented code and add transformation
if __name__ == '__main__':
    rospy.init_node("head_rotator_py")
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()