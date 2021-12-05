#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
import cv2



class imageSubsribe :
    def __init__(self,topic):
        self.cv_object = CvBridge()
        rospy.wait_for_message(topic,CompressedImage,10)
        self.sub = rospy.Subscriber(topic,CompressedImage,self.callback)
        self.image = None
    def callback(self,msg):
        image = self.cv_object.compressed_imgmsg_to_cv2(msg,'bgr8')
        self.image = image

    def get_image(self):
        return self.image


# if __name__ == '__main__':
#     # Initialize a ROS Node
#     rospy.init_node('line_follower_basics')
#     K = Line('/raspicam_node/image')
#     try:
#         rospy.spin()
#     except rospy.ROSException() as e:
#         print(e)
