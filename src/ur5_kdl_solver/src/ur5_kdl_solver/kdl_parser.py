#! /usr/bin/env python

import PyKDL as pk
from urdf_parser_py.urdf import URDF
import numpy as np

class KDLParser(object):
    def __init__(self):
        self.urdf=URDF.from_parameter_server(key='robot_description')

    def euler_to_quat(self,r, p, y):
        sr, sp, sy = np.sin(r/2.0), np.sin(p/2.0), np.sin(y/2.0)
        cr, cp, cy = np.cos(r/2.0), np.cos(p/2.0), np.cos(y/2.0)
        return [sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy,
                cr*cp*cy + sr*sp*sy]

    def urdf_pose_to_kdl_frame(self,pose):
        pos = [0., 0., 0.]
        rot = [0., 0., 0.]
        if pose is not None:
            if pose.position is not None:
                pos = pose.position
            if pose.rotation is not None:
                rot = pose.rotation
        return pk.Frame(pk.Rotation.Quaternion(*self.euler_to_quat(*rot)),
                         pk.Vector(*pos))

    def urdf_joint_to_kdl_joint(self,jnt):
        origin_frame = self.urdf_pose_to_kdl_frame(jnt.origin)
        if jnt.joint_type == 'fixed':
            return pk.Joint(jnt.name, getattr(pk.Joint,"None"))
        
        axis = pk.Vector(*[float(s) for s in jnt.axis])
        if jnt.joint_type == 'revolute':
            return pk.Joint(jnt.name, origin_frame.p,
                             origin_frame.M * axis, pk.Joint.RotAxis)
        if jnt.joint_type == 'continuous':
            return pk.Joint(jnt.name, origin_frame.p,
                             origin_frame.M * axis, pk.Joint.RotAxis)
        if jnt.joint_type == 'prismatic':
            return pk.Joint(jnt.name, origin_frame.p,
                             origin_frame.M * axis, pk.Joint.TransAxis)
        print ("Unknown joint type: %s." % jnt.joint_type)
        return pk.Joint(jnt.name, getattr(pk.Joint,"None"))

    
    def urdf_inertial_to_kdl_rbi(self,i):
        origin = self.urdf_pose_to_kdl_frame(i.origin)
        rbi = pk.RigidBodyInertia(i.mass, origin.p,
                               pk.RotationalInertia(i.inertia.ixx,
                                                     i.inertia.iyy,
                                                     i.inertia.izz,
                                                     i.inertia.ixy,
                                                     i.inertia.ixz,
                                                     i.inertia.iyz))
        return origin.M * rbi
    
    def kdl_tree_from_urdf_model(self,urdf):
        root = urdf.get_root()
        tree = pk.Tree(root)
        def add_children_to_tree(parent):
            if parent in urdf.child_map:
                for joint, child_name in urdf.child_map[parent]:
                    for lidx, link in enumerate(urdf.links):
                        if child_name == link.name:
                            child = urdf.links[lidx]
                            if child.inertial is not None:
                                kdl_inert = self.urdf_inertial_to_kdl_rbi(child.inertial)
                            else:
                                kdl_inert = pk.RigidBodyInertia()
                            for jidx, jnt in enumerate(urdf.joints):
                                if jnt.name == joint:
                                    kdl_jnt = self.urdf_joint_to_kdl_joint(urdf.joints[jidx])
                                    kdl_origin = self.urdf_pose_to_kdl_frame(urdf.joints[jidx].origin)
                                    kdl_sgm = pk.Segment(child_name, kdl_jnt,
                                                      kdl_origin, kdl_inert)
                                    tree.addSegment(kdl_sgm, parent)
                                    add_children_to_tree(child_name)
        add_children_to_tree(root)
        return tree
