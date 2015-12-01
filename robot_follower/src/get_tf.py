#!/usr/bin/env python
import cv2, rospy, roslib, sys
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
roslib.load_manifest('robot_follower')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3

# position = Vector3()
def callback1(data):
    if(data.transforms[0].child_frame_id == 'head_1'):
        position = data.transforms[0].transform.translation
        # print 'x: %f, y: %f, z: %f' % (position.x, position.y, position.z)

image = cv2.
is_image_present = 0
def callback2(data):
    try:
      cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    image = cv_image
    is_image_present = 1
    
def match_head():
    height = 480
    width = 640
    rate = rospy.Rate(10)
    cv2.namedWindow("Match /head_x (joint) to Person", 1)
    while not rospy.is_shutdown():
      if(is_image_present):
        im = self.image
      cv2.circle(im, (position.x, positiom.y), 5, (255, 0, 0))
      cv2.imshow("Match /head_x (joint) to Person", im)
      cv2.waitKey(3)
      rate.sleep()

if __name__ == '__main__':
    rospy.init_node('get_tf', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_color", Image, callback2)
    rospy.Subscriber("/tf", TFMessage, callback1)
    match_head()
    rospy.spin()
