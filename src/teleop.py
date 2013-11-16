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
# 3. Neither the name of the University of Massachusetts Lowell nor the names
#    from of its contributors may be used to endorse or promote products
#    derived this software without specific prior written permission.
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

import baxter_faces

import vis


class HeadMover(object):
    def __init__(self):
        self._head = baxter_interface.Head()
        self.pan_angle = 0

    def set_pose(self):
        self._head.set_pan(self.pan_angle)

    def parse_joy(self, joypad):
        if joypad.joy[0] != 0:
            increment = -joypad.joy[0] / 50
            self.pan_angle += increment
            if abs(self.pan_angle) > 1.57:
                self.pan_angle -= increment
            self.set_pose()


class LimbMover(object):
    def __init__(self, limb):
        self.limb = limb
        self.interface = baxter_interface.Limb(limb)
        self.solver = IKSolver(limb)
        self.last_solve_request_time = rospy.Time.now()
        self.running = True
        self.thread = threading.Thread(target=self._update_thread)
        self.vis = vis.Vis()

    def enable(self):
        self.thread.start()

    def set_target(self, joints):
        self.target_joints = joints

    def _update_req_time(self):
        self.last_solve_request_time = rospy.Time.now()

    def _solver_cooled_down(self):
        time_since_req = rospy.Time.now() - self.last_solve_request_time
        return time_since_req > rospy.Duration(0.05)  # 20 Hz

    def update(self, trigger, gripper_travel):
        self.vis.show_gripper(self.limb, gripper_travel, 0.026, 0.11, 1)

        # Throttle service requests
        if trigger and self._solver_cooled_down():
            self._update_req_time()
            return self.solver.solve()
        return True

    def stop_thread(self):
        if self.thread.is_alive():
            self.running = False
            self.thread.join()

    def _update_thread(self):
        rospy.loginfo("Starting Joint Update Thread: %s\n" % self.limb)
        rate = rospy.Rate(200)
        while not rospy.is_shutdown() and self.running:
            self.interface.set_joint_positions(self.solver.solution)
            rate.sleep()
        rospy.loginfo("Stopped %s" % self.limb)


class IKSolver(object):
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
            return True

        else:
            rospy.logwarn("INVALID POSE for %s" % self.limb)
            return False


class Teleop(object):

    hydra_msg_lock = threading.Lock()
    enabled = False  # We wait until the user presses a button

    def __init__(self):
        global _status_display
        rospy.init_node("baxter_hydra_teleop")
        _status_display = baxter_faces.FaceImage()
        rospy.loginfo("Getting robot state... ")
        self.rs = baxter_interface.RobotEnable()

        self.gripper_left = baxter_interface.Gripper("left")
        self.gripper_right = baxter_interface.Gripper("right")
        self.mover_left = LimbMover("left")
        self.mover_right = LimbMover("right")
        self.mover_head = HeadMover()
        self.happy_count = 0  # Need inertia on how long unhappy is displayed
        self.hydra_msg = Hydra()

        rospy.on_shutdown(self._cleanup)
        sub = rospy.Subscriber("/hydra_calib", Hydra, self._hydra_cb)

        rospy.Timer(rospy.Duration(1.0 / 30), self._main_loop)

        rospy.loginfo(
          "Press left or right button on Hydra to start the teleop")
        while not self.enabled and not rospy.is_shutdown():
            rospy.Rate(10).sleep()
        self.mover_left.enable()
        self.mover_right.enable()
        self.mover_head.set_pose()

    def _reset_grippers(self, event):
        rospy.loginfo('Resetting grippers')
        self.gripper_left.reboot()
        self.gripper_right.reboot()
        self.gripper_left.calibrate()
        self.gripper_right.calibrate()

    def _enable(self):
        rospy.loginfo("Enabling robot... ")
        self.rs.enable()
        rospy.Timer(rospy.Duration(0.1), self._reset_grippers, oneshot=True)
        _status_display.set_image('happy')

    def _hydra_cb(self, msg):
        with self.hydra_msg_lock:
            self.hydra_msg = msg

    def _main_loop(self, event):
        with self.hydra_msg_lock:
            msg = self.hydra_msg

        self._terminate_if_pressed(msg)

        self.mover_left.update(False, msg.paddles[0].trigger)
        self.mover_right.update(False, msg.paddles[1].trigger)

        if not self.enabled:
            self.enabled = (
                msg.paddles[0].buttons[0] or msg.paddles[1].buttons[0])
            return

        if not rospy.is_shutdown():
            happy0 = self.mover_left.update(
                msg.paddles[0].buttons[0], msg.paddles[0].trigger)
            happy1 = self.mover_right.update(
                msg.paddles[1].buttons[0], msg.paddles[1].trigger)
            if happy0 and happy1:
                self.happy_count += 1
                if self.happy_count > 200:
                    _status_display.set_image('happy')
            else:
                self.happy_count = 0
                _status_display.set_image('confused')

            self.mover_head.parse_joy(msg.paddles[0])
            self.gripper_left.set_position(
                100 * (1 - msg.paddles[0].trigger))
            self.gripper_right.set_position(
                100 * (1 - msg.paddles[1].trigger))

    def _terminate_if_pressed(self, hydra):
        if(sum(hydra.paddles[0].buttons[1:] + hydra.paddles[1].buttons[1:])):
            self._cleanup()

    def _cleanup(self):
        rospy.loginfo("Disabling robot... ")
        self.rs.disable()
        self.mover_left.stop_thread()
        self.mover_right.stop_thread()


if __name__ == '__main__':
    Teleop()
