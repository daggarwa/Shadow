#!/usr/bin/env python
import rospy
import roslib
import sys
from tf2_msgs.msg import TFMessage
roslib.load_manifest('robot_follower')
from std_msgs.msg import String

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if(data.transforms[0].child_frame_id == 'head_1'):
        print 'x: %f, y: %f, z: %f'% (data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z)
    
    #print 'hello'
    
def get_tf():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'get_tf' node so that multiple get_tfs can
    # run simultaneously.
    rospy.init_node('get_tf', anonymous=True)

    rospy.Subscriber("/tf", TFMessage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    get_tf()
