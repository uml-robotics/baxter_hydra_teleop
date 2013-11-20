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

import roslib
roslib.load_manifest('baxter_hydra_teleop')
import rospy
import tf

from geometry_msgs.msg import (
    Pose,
    Transform,
    Quaternion,
)


class GoalTransform(object):
    """ Publish an additional transforms constrained in some way.

    Should allow to make it easier to add teleoperation constraints,
    e.g. lock the control plane, or change motion scaling.
    """

    def __init__(self, limb):
        self.modes = {
                      "identity": self._identity,
                      "orientation": self._orientation_lock,
                      "plane": self._plane_lock
                      }
        self.set_mode("identity")
        self.br = tf.TransformBroadcaster()
        self.plane = Transform()
        self.plane.rotation.w = 1
        self.limb = limb
        self.orientation = Quaternion(0, 0, 0, 1)
        self.orientation_lock_frame = "base"
        self.tf_listener = tf.TransformListener()
        """
        _tf_offset is used to add an offset to the stamp of published
        transforms. It is a hack needed to IK nodes on Baxter happy, and
        would be different for different systems.
        """
        self._tf_offset = 0.1

        self.plane = Pose()

    def set_mode(self, mode):
        self.updater = self.modes[mode]

    def update(self):
        self.updater()

    def _identity(self):
        """ No transformation, 1 to 1 mapping """
        self.br.sendTransform(
           (0, 0, 0),
           (0, 0, 0, 1),
           rospy.Time.now() + rospy.Duration(self._tf_offset),
            self.limb + "_gripper_goal",
           "hydra_" + self.limb + "_grab")

    def _orientation_lock(self):
        """ Lock the orientation """
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                   self.orientation_lock_frame,
                   'hydra_' + self.limb + '_grab',
                   rospy.Time(0))
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            print e
            return

        self.br.sendTransform(
           trans,
           (
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w,
           ),
           rospy.Time.now() + rospy.Duration(self._tf_offset),
           self.limb + "_gripper_goal",
           self.orientation_lock_frame)

    def _plane_lock(self):
        """ Lock the orientation """
        pass
