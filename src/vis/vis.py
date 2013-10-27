import roslib
roslib.load_manifest('baxter_hydra_teleop')
import rospy

from std_msgs.msg import Header

from visualization_msgs.msg import Marker


class Vis(object):
    def __init__(self):
        self.pub = rospy.Publisher(
            '/visualization_marker', Marker)
        self.last_time = rospy.Time.now()

    def show_gripper(self, limb):
        # Throttle
        if (rospy.Time.now() - self.last_time) < rospy.Duration(0.05):
            return
        self.last_time = rospy.Time.now()

        hdr = Header(
            #stamp=rospy.Time.now(), frame_id='left_gripper')
            stamp=rospy.Time.now(), frame_id='hydra_' + limb + '_grab')

        # Gripper projection
        msg = Marker(header=hdr, ns=limb)
        msg.scale.x = 0.009
        msg.scale.y = 0.009
        msg.scale.z = 0.11
        msg.type = Marker.CYLINDER
        msg.color.b = 0.5
        msg.color.a = 0.8
        msg.pose.position.z = -0.055
        msg.pose.position.y = -0.025
        msg.pose.orientation.x = 1
        self.pub.publish(msg)
        msg.id += 1
        msg.pose.position.y = +0.025
        self.pub.publish(msg)

        msg.type = Marker.MESH_RESOURCE
        msg.id += 1
        msg.scale.x = 1
        msg.scale.y = 1
        msg.scale.z = 1
        msg.pose.position.x = -0.04
        msg.pose.position.y = 0
        msg.pose.position.z = -0.01

        msg.lifetime = rospy.Duration()
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        msg.color.r = 0.2
        msg.color.g = 0.2
        msg.color.b = 0.2
        msg.color.a = 0.5
        msg.mesh_resource = 'package://baxter_hydra_teleop/meshes/hydra.stl'
        self.pub.publish(msg)
