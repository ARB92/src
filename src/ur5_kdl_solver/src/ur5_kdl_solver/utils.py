#! /usr/bin/env python
from geometry_msgs.msg import Pose

CONSTANT = 10

def calculate_new_ee_pose(current_ee_pose,force):
    position = current_ee_pose[0]
    if force.axis == 'x':
        position[0] =+ force.magnitude*CONSTANT
    elif force.axis == 'y':
        position[1] =+ force.magnitude*CONSTANT
    else:
        position[2] =+ force.magnitude*CONSTANT
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = current_ee_pose[1][0]
    pose.orientation.y = current_ee_pose[1][1]
    pose.orientation.z = current_ee_pose[1][2]
    pose.orientation.w = current_ee_pose[1][3]
    return pose.position, pose.orientation
