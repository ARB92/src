#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from admittance_controller_smach.states import Init, Busy, Stopping

def main():
    rospy.init_node('admittance_controller_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Ready','Busy'])
    sm.userdata.bAdmittance = rospy.wait_for_message('/random_signal',Bool)

    # Open the container
    with sm:
            # Add states to the container
        smach.StateMachine.add('INIT', Init(), 
                               transitions={'outcome1':'BUSY', 
                                            'outcome2':'Ready'},
                               remapping={'init_in':'bAdmittance'})
        smach.StateMachine.add('BUSY', Busy(), 
                               transitions={'outcome1':'Busy',
                                   'outcome2':'STOPPING'},
                               remapping={'busy_in':'bAdmittance'})
        smach.StateMachine.add('STOPPING', Stopping(), 
                               transitions={'outcome1':'INIT'})


    outcome = sm.execute()


if __name__ == '__main__':
    main()

