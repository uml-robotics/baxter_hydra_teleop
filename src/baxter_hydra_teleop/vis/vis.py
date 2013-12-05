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


class VisMarker(object):
    _id = -1

    def new_id(self):
        VisMarker._id += 1
        return VisMarker._id

    def publish(self, publisher):
        self.msg.header.stamp = rospy.Time.now()
        publisher.publish(self.msg)


class HydraVis(VisMarker):
    def __init__(self, limb):
        hdr = Header(frame_id='hydra_' + limb + '_grab')
        self.msg = Marker(
            id=self.new_id(),
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


class GripperVis(VisMarker):
    def __init__(self, limb, attached_to_hydra=True,
                 width=0.026, height=0.11, position=1):
        if attached_to_hydra:
            frame = 'hydra_' + limb + '_grab'
            color = ColorRGBA(0.1, 0.1, 0.1, 0.4)
            pos = Point(0, 0, -0.055)
        else:
            frame = limb + '_gripper'
            color = ColorRGBA(0, 0, 0.1, 0.8)
            pos = Point(0, 0, 0.04)
        hdr = Header(frame_id=frame)

        self.width = width
        self.height = height
        self.position = position

        self.spacing = 0.075 / 8  # Gripper attachment spacing
        self.max_gripper_travel = 0.022

        self.id0 = self.new_id()
        self.id1 = self.new_id()
        self.msg = Marker(
            header=hdr,
            ns=limb,
            type=Marker.CUBE,
            scale=Vector3(0.009, 0.009, self.height),
            color=color,
            pose=Pose(
                pos,
                Quaternion(1, 0, 0, 0)
            )
        )

    def publish(self, publisher, travel=1):
        self.msg.header.stamp = rospy.Time.now()
        gripper_offset = (self.width + self.spacing * (self.position - 1)
                          - self.max_gripper_travel * travel)
        self.msg.id = self.id0
        self.msg.pose.position.y = -gripper_offset
        publisher.publish(self.msg)
        self.msg.id = self.id1
        self.msg.pose.position.y = +gripper_offset
        publisher.publish(self.msg)


class Vis(object):
    def __init__(self, limb):
        self.pub = rospy.Publisher(
            '/visualization_marker', Marker)
        self.last_time = rospy.Time.now()
        self.hydra = HydraVis(limb)
        self.gripper = GripperVis(limb, False)
        self.gripper_hydra = GripperVis(limb)

    def show_gripper(self, travel):
        # Throttle
        if (rospy.Time.now() - self.last_time) < rospy.Duration(0.05):
            return
        self.last_time = rospy.Time.now()

        self.hydra.publish(self.pub)
        self.gripper.publish(self.pub, travel)
        self.gripper_hydra.publish(self.pub, travel)
