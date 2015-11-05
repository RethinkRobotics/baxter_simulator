#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class PickAndPlace(object):
    def __init__(self, limb):
        self._limb_name = limb
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self._joint_names = '{0}_w0 {0}_w1 {0}_w2 {0}_e0 {0}_e1 {0}_s0 {0}_s1'.format(limb).split()
        self._neutral_angles = dict(zip(self._joint_names, [0]*7)) 

    def move_to_start(self, start_angles):
        print("Moving the {} arm to neutral pose...".format(self._limb_name))
        self.set_neutral_angles(start_angles)
        self._limb.move_to_joint_positions(start_angles)
        self._gripper.open()
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        print resp
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def pick(self, pose):
        # servo to 5 cm above pose
        pre_pose = pose
        pre_pose.position.z = pre_pose.position.z + 0.05
        joint_angles = self.ik_request(pre_pose)
        self._limb.move_to_joint_positions(joint_angles)
        # open the gripper
        self._gripper.open()
        # servo down 5 cm (to original pose)
        joint_angles = self.ik_request(pose)
        self._limb.move_to_joint_positions(joint_angles)
        # close gripper
        self._gripper.close()
        self.retract()

    def set_neutral_angles(self, neutral_angles):
        self._neutral_angles = neutral_angles

    def retract(self):
        # servo to 5 cm up from starting pose
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + 0.05
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo to neutral pose
        #self._limb.move_to_neutral()
        self._limb.move_to_joint_positions(self._neutral_angles)

    def place(self, pose):
        # servo to 5 cm above pose
        pre_pose = pose
        pre_pose.position.z = pre_pose.position.z + 0.05
        joint_angles = self.ik_request(pre_pose)
        self._limb.move_to_joint_positions(joint_angles)
        # servo down 5 cm
        joint_angles = self.ik_request(pose)
        self._limb.move_to_joint_positions(joint_angles)
        # open the gripper
        self._gripper.open()
        self.retract()


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    rospy.init_node("ik_pick_and_place_demo")
    limb = 'left'
    pnp = PickAndPlace(limb)
    block_poses = list()
    neutral_angles = {'left_w0': 0.6699952259595108, 'left_w1': 1.030009435085784, 'left_w2': -0.4999997247485215, 'left_e0': -1.189968899785275, 'left_e1': 1.9400238130755056, 'left_s0': -0.08000397926829805, 'left_s1': -0.9999781166910306}
    overhead_orientation = Quaternion(
            x=-0.0249590815779,
            y=0.999649402929,
            z=0.00737916180073,
            w=0.00486450832011)
    overhead_orientation = Quaternion(
    x= 0.140764818367,
    y= 0.989646742913,
    z= 0.0116553763534,
    w= 0.0254704211511)
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=-0.03),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.9, y=0.0, z=-0.03),
        orientation=overhead_orientation))
    pnp.move_to_start(neutral_angles)
    idx = 0
    while not rospy.is_shutdown():
        pnp.pick(block_poses[idx])
        idx = idx+1 % len(block_poses)
        pnp.place(block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())
