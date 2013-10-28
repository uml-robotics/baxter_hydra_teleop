import roslib
roslib.load_manifest('baxter_hydra_teleop')
import rospy

from std_msgs.msg import Header

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import (
    Vector3,
    Pose,
    Point,
    Quaternion
)


class Vis(object):
    def __init__(self):
        self.pub = rospy.Publisher(
            '/visualization_marker', Marker)
        self.last_time = rospy.Time.now()

    def show_gripper(self, limb, travel, width=0.026, height=0.11, position=1):
    #def show_gripper(self, limb):
        # Throttle
        if (rospy.Time.now() - self.last_time) < rospy.Duration(0.05):
            return
        self.last_time = rospy.Time.now()

        counter = 0

        hdr = Header(
            stamp=rospy.Time.now(), frame_id='hydra_' + limb + '_grab')

        spacing = 0.075 / 8  # Gripper attachment spacing
        max_gripper_travel = 0.022
        gripper_offset = (
            width + spacing * (position - 1) - max_gripper_travel * travel)

        # Gripper projection
        msg = Marker(
            id=counter,
            header=hdr,
            ns=limb,
            type=Marker.CUBE,
            scale=Vector3(0.009, 0.009, height),
            color=ColorRGBA(0, 0, 0.1, 0.8),
            pose=Pose(
                Point(0, -gripper_offset, -0.055),
                Quaternion(1, 0, 0, 0)
            )
        )
        self.pub.publish(msg)

        counter += 1
        msg.id = counter
        msg.pose.position.y = +gripper_offset
        self.pub.publish(msg)

        counter += 1
        msg = Marker(
            id=counter,
            header=hdr,
            ns=limb,
            type=Marker.MESH_RESOURCE,
            mesh_resource='package://baxter_hydra_teleop/meshes/hydra.stl',
            scale=Vector3(1, 1, 1),
            color=ColorRGBA(0.2, 0.2, 0.2, 0.5),
            pose=Pose(
                Point(-0.04, 0, -0.01),
                Quaternion(0, 0, 0, 1)
            )
        )
        self.pub.publish(msg)
