#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

import tf.transformations as tft

from iri_laser_icp.srv import GetRelativePose

class TestLaserICP:
    def __init__(self):
        rospy.init_node('test_laser_icp', anonymous=True)

        self._sub = rospy.Subscriber('scan', LaserScan, self._callback)

        self._first = True
        self._pose = PoseWithCovarianceStamped()
        self._scan1 = LaserScan()
        self._scan2 = LaserScan()
        self._prior = Pose()

    def _get_pose_client(self):
        try:
            pose_srv = rospy.ServiceProxy('laser_icp_server/get_relative_pose', GetRelativePose)
            pose_srv.wait_for_service()

            res = pose_srv(self._scan0, self._scan1, self._prior)

            if res.success:
                return res.pose_rel.pose.pose
            else:
                rospy.logwarn('Failed to find relative pose')
                return None
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)

    def _callback(self, msg):
        if self._first:
            self._scan0 = msg
            self._first = False
        else:
            self._scan1 = msg
            pose = self._get_pose_client()

            if pose != None:
                q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                rpy = tft.euler_from_quaternion(q)
                yaw = rpy[2]

                rospy.loginfo('Pose (x, y, theta) = (%f, %f, %f)', pose.position.x, pose.position.y, yaw)

            self._scan0 = self._scan1

def main():
    tli = TestLaserICP()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

