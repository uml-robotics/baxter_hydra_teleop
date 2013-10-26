#!/usr/bin/env python

# Copyright (c) 2013, University Of Massachusetts Lowell
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

  Baxter Teleoperation using Razer Hydra

"""
import argparse
import sys
import threading

import roslib
roslib.load_manifest('baxter_hydra_teleop')
import rospy

from razer_hydra.msg import Hydra

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Transform,
)
from std_msgs.msg import Header
from baxter_msgs.srv import SolvePositionIK
from baxter_msgs.srv import SolvePositionIKRequest
import baxter_interface


class LimbMover:
    def __init__(self, limb):
        self.limb = limb
        self.interface = baxter_interface.Limb(limb)
        self.solver = IKSolver(limb)
        self.last_solve_request_time = rospy.Time.now()
        self.running = True
        self.thread = threading.Thread(target=self.update_thread)

    def enable(self):
        self.thread.start()

    def set_target(self, joints):
        self.target_joints = joints

    def update_req_time(self):
        self.last_solve_request_time = rospy.Time.now()

    def solver_cooled_down(self):
        time_since_req = rospy.Time.now() - self.last_solve_request_time
        return time_since_req > rospy.Duration(0.05)

    def parse_joy(self, joypad):
        # Throttle service requests
        if joypad.buttons[0] and self.solver_cooled_down():
            self.update_req_time()
            self.solver.solve()

    def stop_thread(self):
        self.running = False
        self.thread.join()

    def update_thread(self):
        rospy.loginfo("Starting Joint Update Thread: %s\n" % self.limb)
        rate = rospy.Rate(200)
        while not rospy.is_shutdown() and self.running:
            self.interface.set_joint_positions(self.solver.solution)
            rate.sleep()
        rospy.loginfo("Stopped %s" % self.limb)


class IKSolver:
    def __init__(self, limb):
        self.limb = limb
        ns = "/sdk/robot/limb/" + self.limb + "/solve_ik_position"
        rospy.wait_for_service(ns)
        self.iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        self.solution = dict()
        self.mapping = [
            # Human-like mapping (front of camera  = front of the wrist)
            Quaternion(
                x=0,
                y=0.7071067811865475244,  # sqrt(0.5)
                z=0,
                w=0.7071067811865475244  # sqrt(0.5)
            # Camera is pointing down
            ), Quaternion(
                x=0,
                y=1,
                z=0,
                w=0
            ),
        ]

    def solve(self):
        ikreq = SolvePositionIKRequest()
        hdr = Header(
            stamp=rospy.Time.now(), frame_id='hydra_' + self.limb + '_grab')
        pose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=0, y=0, z=0),
                orientation=self.mapping[1]
            ),
        )

        ikreq.pose_stamp.append(pose)
        try:
            resp = self.iksvc(ikreq)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % (e,))

        if (resp.isValid[0]):
            self.solution = dict(
                zip(resp.joints[0].names, resp.joints[0].angles))
            rospy.loginfo("Solution Found, %s" % self.limb, self.solution)

        else:
            rospy.logwarn("INVALID POSE for %s" % self.limb)


class Teleop:
    def __init__(self):
        rospy.init_node("baxter_hydra_teleop")
        rospy.loginfo("Getting robot state... ")
        self.rs = baxter_interface.RobotEnable()

        self.gripper_left = baxter_interface.Gripper("left")
        self.gripper_right = baxter_interface.Gripper("right")
        self.mover_left = LimbMover("left")
        self.mover_right = LimbMover("right")

        rospy.on_shutdown(self.cleanup)
        sub = rospy.Subscriber("/hydra_calib", Hydra, self.hydra_cb)

        rospy.loginfo(
          "Press left or right button on Hydra to start the teleop")
        self.enabled = False  # We wait until the user presses a button
        while not self.enabled:
            pass
        rospy.loginfo("Enabling robot... ")
        self.rs.enable()
        self.mover_left.enable()
        self.mover_right.enable()

    def hydra_cb(self, msg):
        if not self.enabled:
            self.enabled = (
                msg.paddles[0].buttons[0] or msg.paddles[1].buttons[0])
            return
        map(self.stop_on_buttons, msg.paddles)
        if not rospy.is_shutdown():
            self.mover_left.parse_joy(msg.paddles[0])
            self.mover_right.parse_joy(msg.paddles[1])
            self.gripper_left.set_position(
                100 * (1 - msg.paddles[0].trigger))
            self.gripper_right.set_position(
                100 * (1 - msg.paddles[1].trigger))

    def stop_on_buttons(self, val):
        for x in val.buttons[1:]:
            if x:
                self.cleanup()
                return

    def cleanup(self):
        self.mover_left.stop_thread()
        self.mover_right.stop_thread()
        rospy.loginfo("Disabling robot... ")
        self.rs.disable()


if __name__ == '__main__':
    Teleop()
