#!/usr/bin/env python

import rospy
from iri_laser_icp.srv import GetRelativePose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

import sys

class test_laser_icp:

  def __init__(self):
    rospy.loginfo("initiating node...")
    rospy.init_node('test_laser_icp', anonymous=True)
    self.scan_subs = rospy.Subscriber("scan", LaserScan, self.callback)
    self.first = True
    self.pose = PoseWithCovarianceStamped()
    self.scan1 = LaserScan()
    self.scan2 = LaserScan()

  def get_pose_client(self):
      rospy.loginfo("waiting to get service...")
      rospy.wait_for_service('test_icp_server/get_relative_pose')
      try:
          get_pose = rospy.ServiceProxy('test_icp_server/get_relative_pose', GetRelativePose)
          respose = get_pose(self.scan1, self.scan2)
          self.pose = respose.pose_rel
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  def callback(self, laserscan):
      rospy.loginfo("le callback")
      if(self.first):
        self.scan1 = laserscan
        self.first = False
      else:
        self.scan2 = laserscan
        self.get_pose_client()
        rospy.loginfo(" Pose: x,y %f %f ",self.pose.pose.pose.position.x,self.pose.pose.pose.position.y)
        self.scan1 = self.scan2

def main(args):
      li = test_laser_icp()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)

