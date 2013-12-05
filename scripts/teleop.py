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

import threading

import roslib
roslib.load_manifest('baxter_hydra_teleop')
import rospy

from razer_hydra.msg import Hydra
import baxter_interface
import baxter_faces
from baxter_hydra_teleop.head_mover import HeadMover
from baxter_hydra_teleop.limb_mover import LimbMover


class Teleop(object):

    def __init__(self):
        rospy.init_node("baxter_hydra_teleop")
        self.status_display = baxter_faces.FaceImage()
        rospy.loginfo("Getting robot state... ")
        self.rs = baxter_interface.RobotEnable()
        self.hydra_msg = Hydra()
        self.hydra_msg_lock = threading.Lock()

        self.gripper_left = baxter_interface.Gripper("left")
        self.gripper_right = baxter_interface.Gripper("right")
        self.mover_left = LimbMover("left")
        self.mover_right = LimbMover("right")
        self.mover_head = HeadMover()
        self.happy_count = 0  # Need inertia on how long unhappy is displayed
        self.hydra_msg = Hydra()

        rospy.on_shutdown(self._cleanup)
        rospy.Subscriber("/hydra_calib", Hydra, self._hydra_cb)

        rospy.Timer(rospy.Duration(1.0 / 30), self._main_loop)

        rospy.loginfo(
          "Press left or right button on Hydra to start the teleop")
        while not self.rs.state().enabled and not rospy.is_shutdown():
            rospy.Rate(10).sleep()
        self.mover_left.enable()
        self.mover_right.enable()
        self.mover_head.set_pose()

    def _reset_gripper(self, gripper):
        rospy.loginfo("Resetting %s" % gripper.name)
        gripper.reboot()
        gripper.set_moving_force(10)
        gripper.set_holding_force(20)
        gripper.set_dead_band(5)
        if not gripper.calibrated():
            gripper.calibrate()
        gripper.command_position(100)

    def _reset_grippers(self, event):
        rospy.loginfo('Resetting grippers')
        self._reset_gripper(self.gripper_right)
        self._reset_gripper(self.gripper_left)

    def _enable(self):
        rospy.loginfo("Enabling robot... ")
        self.rs.enable()
        rospy.Timer(rospy.Duration(0.1), self._reset_grippers, oneshot=True)
        self.status_display.set_image('happy')

    def _hydra_cb(self, msg):
        with self.hydra_msg_lock:
            self.hydra_msg = msg

    def _main_loop(self, event):
        if self.rs.state().estop_button == 1:
            self.status_display.set_image('dead')
            return
        else:
            if not self.rs.state().enabled:
                self.status_display.set_image('indifferent')

        with self.hydra_msg_lock:
            msg = self.hydra_msg

        self._terminate_if_pressed(msg)

        self.mover_left.update(False, 1 - self.gripper_left.position() / 100)
        self.mover_right.update(False, 1 - self.gripper_right.position() / 100)

        if not self.rs.state().enabled:
            if msg.paddles[0].buttons[0] or msg.paddles[1].buttons[0]:
                self._enable()
            return

        if not rospy.is_shutdown():
            happy0 = self.mover_left.update(
                msg.paddles[0].buttons[0],
                1 - self.gripper_left.position() / 100)
            happy1 = self.mover_right.update(
                msg.paddles[1].buttons[0],
                1 - self.gripper_right.position() / 100)
            if happy0 and happy1:
                self.happy_count += 1
                if self.happy_count > 200:
                    self.status_display.set_image('happy')
            else:
                self.happy_count = 0
                self.status_display.set_image('confused')

            self.mover_head.parse_joy(msg.paddles[0])
            self.gripper_left.command_position(
                100 * (1 - msg.paddles[0].trigger))
            self.gripper_right.command_position(
                100 * (1 - msg.paddles[1].trigger))

    def _terminate_if_pressed(self, hydra):
        if(hydra.paddles[0].buttons[5] or hydra.paddles[1].buttons[5]):
            self._cleanup()

    def _cleanup(self):
        rospy.loginfo("Disabling robot... ")
        self.rs.disable()
        self.mover_left.stop_thread()
        self.mover_right.stop_thread()


if __name__ == '__main__':
    Teleop()
