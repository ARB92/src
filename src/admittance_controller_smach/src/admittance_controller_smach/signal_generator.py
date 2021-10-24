#! /usr/bin/env python

import rospy
import random
from std_msgs.msg import Bool

def random_signal_generator():
    rospy.init_node('random_signal_generator')
    pub = rospy.Publisher('random_signal',Bool)
    signal = [True,False]
    while not rospy.is_shutdown():
        msg = random.choice(signal)
        pub.publish(msg)
        rospy.sleep(5)

