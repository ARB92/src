#! /usr/bin/env python

import rospy
import math
import actionlib
from control_msgs.msg import JointTrajectoryActionGoal, FollowJointTrajectoryActionGoal,JointTrajectoryGoal, FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint


class JointPublisher():
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',FollowJointTrajectoryAction)
        self.joints = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    
    def publish_joint_values(self,joint_values):
        goal  = FollowJointTrajectoryGoal()
        points = JointTrajectoryPoint()
        goal.trajectory.joint_names = self.joints
        points.positions = joint_values
        points.velocities = [0.001]*6
        points.time_from_start = rospy.Duration(0.01)
        goal.trajectory.points = [points]
        goal.trajectory.header.stamp = rospy.Time.now()
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()

def main():
    rospy.init_node("joint_publisher")
    jp = JointPublisher()
    for number in range(0,100):
        value = -3.14*(math.sin(number/100))
        joint_values = [0,value,0,value,0,0]
        jp.publish_joint_values(joint_values)
        rospy.sleep(1)

if __name__ == "__main__":
    main()
