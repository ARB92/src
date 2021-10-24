#! /usr/bin/env python

import PyKDL as pk
import numpy as np

class KDLKinematics(object):
    def __init__(self, chain):
        """kinematic solver for flexible joints"""
        self.chain = chain
        self.number_of_joints = chain.getNrOfJoints()

        # KDL solvers
        self.fk_position_kdl = pk.ChainFkSolverPos_recursive(self.chain)
        self.ik_velocity_kdl = pk.ChainIkSolverVel_pinv(self.chain)
        self.ik_position_kdl = pk.ChainIkSolverPos_NR(self.chain,
                                                         self.fk_position_kdl,
                                                         self.ik_velocity_kdl)

    def joint_values_to_kdl_array(self, joint_values):
        kdl_array = pk.JntArray(self.number_of_joints)
        for joint in range(self.number_of_joints):
            kdl_array[joint] = joint_values[joint]
        return kdl_array

    def forward_position_kinematics(self, joint_values=None):
        end_frame = pk.Frame()
        kdl_array = self.joint_values_to_kdl_array(joint_values)
        self.fk_position_kdl.JntToCart(kdl_array, end_frame)
        pos = end_frame.p
        rot = pk.Rotation(end_frame.M)
        rot = rot.GetQuaternion()

        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

    def inverse_kinematics(self, position, orientation, initial_angles=[0, 0, 0, 0, 0, 0], joint_limits=False):
        pos = pk.Vector(position.x, position.y, position.z)
        rot = pk.Rotation()
        rot = rot.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        goal_pose = pk.Frame(rot, pos)
        result_angles = pk.JntArray(self.number_of_joints)
        initial_angles_kdl_array = self.joint_values_to_kdl_array(initial_angles)
        self.ik_position_kdl.CartToJnt(initial_angles_kdl_array, goal_pose, result_angles)
        result = np.array(list(result_angles))
        return result
