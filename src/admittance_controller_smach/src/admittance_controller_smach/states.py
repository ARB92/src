#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['init_in'])

    def execute(self, userdata):
        if userdata.init_in.data == False:
            print(type(userdata.init_in))
            rospy.loginfo("Robot is Ready")
            return 'outcome2'
        else:
            print(type(userdata.init_in))
            print(userdata.init_in)
            rospy.loginfo("Robot is transitioning to Busy state")
            return 'outcome2'


class Busy(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['busy_in'])
        
    def execute(self, userdata):
        if userdata.busy_in.data:
            rospy.loginfo('Robot is Busy')
            return 'outcome1'
        else:
            return 'outcome2'
        
class Stopping(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'])
        
    def execute(self):
        rospy.loginfo('Deactivating the admittance controller')
        rospy.sleep(2)
        return 'outcome1'

