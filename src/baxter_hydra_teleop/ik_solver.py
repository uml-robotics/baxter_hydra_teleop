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

import roslib
roslib.load_manifest('baxter_hydra_teleop')
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK
from baxter_core_msgs.srv import SolvePositionIKRequest
import signal 

import time
import threading
class ServiceTimeouter(object):
    """ Ros services cannot be timed out. Occasionally the IK solver would take
        up to 5 seconds to respond. This is a workaround class. """
    def __init__(self, srv, param):
        self.srv = srv
        self.param = param
        self.timeout = 0.5
        self.retval = None
        self.returned = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._call_thread)
    def _call_thread(self):
        try:
            self.retval = self.srv(self.param)
            self.returned = True
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % (e,))
        except AttributeError:
            rospy.loginfo("Socket.close() exception. Socket has become 'None'")
    def call(self):
        self.thread.start()
        timeout = time.time() + self.timeout
        while time.time() < timeout and self.thread.isAlive():
            time.sleep(0.001)
        if not self.returned:
            print "timed out"
            return None
        return self.retval
        

#import subprocess, threading
#
#class Command(object):
#    def __init__(self, cmd):
#        self.cmd = cmd
#        self.process = None
#
#    def run(self, timeout):
#        def target():
#            print 'Thread started'
#            self.process = subprocess.Popen(self.cmd, shell=True)
#            self.process.communicate()
#            print 'Thread finished'
#
#        thread = threading.Thread(target=target)
#        thread.start()
#
#        thread.join(timeout)
#        if thread.is_alive():
#            print 'Terminating process'
#            self.process.terminate()
#            thread.join()
#        print self.process.returncode


def timeout(func, args=(), kwargs={}, timeout_duration=1, default=None):
#    import signal

    class TimeoutError(Exception):
        pass

    def handler(signum, frame):
        raise TimeoutError()

    # set the timeout handler
    signal.signal(signal.SIGALRM, handler) 
    signal.alarm(timeout_duration)
    try:
        result = func(*args, **kwargs)
    except TimeoutError as exc:
        result = default
    finally:
        signal.alarm(0)

    return result

class IKSolver(object):
    def __init__(self, limb):
        self.limb = limb
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
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
            stamp=rospy.Time.now(), frame_id=self.limb + '_gripper_goal')
        pose = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(x=0, y=0, z=0),
                orientation=self.mapping[1]
            ),
        )

        ikreq.pose_stamp.append(pose)
        resp=None
        try:
#            resp = self.iksvc(ikreq)
            resp = ServiceTimeouter(self.iksvc, ikreq).call()
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % (e,))

        if (resp is not None and resp.isValid[0]):
            self.solution = dict(
                zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo("Solution Found, %s" % self.limb, self.solution)
            return True

        else:
            rospy.logwarn("INVALID POSE for %s" % self.limb)
            return False
