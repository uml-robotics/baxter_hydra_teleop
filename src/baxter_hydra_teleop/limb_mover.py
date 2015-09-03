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

import threading

import roslib
roslib.load_manifest('baxter_hydra_teleop')
import rospy

import baxter_interface
from baxter_hydra_teleop.ik_solver import IKSolver
from baxter_hydra_teleop.goal_transform import GoalTransform
from baxter_hydra_teleop.vis.vis import Vis


class LimbMover(object):
    def __init__(self, limb):
        self.limb = limb
        self.interface = baxter_interface.Limb(limb)
        self.interface.set_joint_position_speed(0.5)
        self.solver = IKSolver(limb)
        self.last_solve_request_time = rospy.Time.now()
        self.running = True
        self.thread = threading.Thread(target=self._update_thread)
        self.vis = Vis(limb)
        self.goal_transform = GoalTransform(limb)
        self._lock = threading.Lock()

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
        
        self.vis.show_gripper(gripper_travel)
        self.goal_transform.update()
        
        # Throttle service requests
        if trigger and self._solver_cooled_down():
            self._update_req_time()
            with self._lock:            
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
            with self._lock:
                self.interface.set_joint_positions(self.solver.solution)
            rate.sleep()
        rospy.loginfo("Stopped %s" % self.limb)
