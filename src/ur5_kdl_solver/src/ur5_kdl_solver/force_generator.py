#! /usr/bin/env python

import rospy
import random
from task_msgs.msg import Force

def random_force_generator():
    rospy.init_node('random_force_generator')
    pub = rospy.Publisher('random_force',Force)
    msg = Force()
    axis = ['x','y','z']
    while not rospy.is_shutdown():
        msg.axis = random.choice(axis)
        msg.magnitude = random.randint(10,100)
        pub.publish(msg)
        rospy.sleep(5)

