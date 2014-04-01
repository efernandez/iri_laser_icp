#!/usr/bin/env python
import roslib; roslib.load_manifest('iri_laser_icp')
import rospy
import os
import sys
from iri_laser_icp.srv import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped


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

# if __name__ == '__main__':
#   rospy.loginfo("test laser icp")
#   while not rospy.is_shutdown():
#     try:
#         compute_icp()
#     except rospy.ROSInterruptException:
#         pass



# class test_vision_node:

#     def __init__(self):
#         rospy.init_node('test_vision_node')

#         """ Give the OpenCV display window a name. """
#         self.cv_window_name = "OpenCV Image"

#         """ Create the window and make it re-sizeable (second parameter = 0) """
#         cv.NamedWindow(self.cv_window_name, 0)

#         """ Create the cv_bridge object """
#         self.bridge = CvBridge()

#         """ Subscribe to the raw camera image topic """
#         self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)

#     def callback(self, data):
#         try:
#             """ Convert the raw image to OpenCV format """
#             cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
#         except CvBridgeError, e:
#           print e
  
        
#         """ Get the width and height of the image """
#         (width, height) = cv.GetSize(cv_image)

#         """ Overlay some text onto the image display """
#         text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 2, 2)
#         cv.PutText(cv_image, "OpenCV Image", (50, height / 2), text_font, cv.RGB(255, 255, 0))
  
#         """ Refresh the image on the screen """
#         cv.ShowImage(self.cv_window_name, cv_image)
#         cv.WaitKey(3)

# def main(args):
#       vn = test_vision_node()
#       try:
#         rospy.spin()
#       except KeyboardInterrupt:
#         print "Shutting down vison node."
#       cv.DestroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)



    # std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# geometry_msgs/PoseWithCovariance pose
#   geometry_msgs/Pose pose
#     geometry_msgs/Point position
#       float64 x
#       float64 y
#       float64 z
#     geometry_msgs/Quaternion orientation
#       float64 x
#       float64 y
#       float64 z
#       float64 w
#   float64[36] covariance