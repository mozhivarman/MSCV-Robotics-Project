#!/usr/bin/env python



#####
# The code blow is helps in finding the values of 
# the image coordinated to be croped from the
# image topic subscribed with the dynamic reconfigurer 
#####


# import the required packages
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
# custom dynamic reconfigurer for this package 
from turtlebot3_autorace_lane.cfg import ImageProjectionParametersConfig


#image Projection class
class ImageProjection():
    """
    Get the image coordinates from the dynamic reconfigurer and draws lines for
    the image coordinats to cropped.
    """
    def __init__(self):

        # intlizes the coordinates
        self.top_x1 = rospy.get_param("/camera/extrinsic_camera_calibration/top_x11", 0)
        self.top_y1 =  rospy.get_param("/camera/extrinsic_camera_calibration/top_y1", 415)
        self.bottom_x1 = rospy.get_param("/camera/extrinsic_camera_calibration/bottom_x1", 640)
        self.bottom_y1 =     rospy.get_param("/camera/extrinsic_camera_calibration/bottom_y1", 415)
        
        self.top_x2 = rospy.get_param("/camera/extrinsic_camera_calibration/top_x2", 0)
        self.top_y2 =  rospy.get_param("/camera/extrinsic_camera_calibration/top_y2", 480)
        self.bottom_x2 = rospy.get_param("/camera/extrinsic_camera_calibration/bottom_x2", 640)
        self.bottom_y2 =     rospy.get_param("/camera/extrinsic_camera_calibration/bottom_y2", 480)
        srv_image_projection = Server(ImageProjectionParametersConfig, self.cbGetImageProjectionParam)

        # image topic subscibred. 
        self.sub_image_original = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.cbImageProjection, queue_size=1)
        # image topic to publish the resulting image
        self.pub_image_calib = rospy.Publisher('/camera/image_calib/compressed', CompressedImage, queue_size=1)
        # cvbridge between ros Image topic and opencv image topic
        self.cvBridge = CvBridge()



    def cbGetImageProjectionParam(self, config, level):
        """
        Update the  dynamic reconfigure parameeters when changed in dynamic reconfigurer.
        """
        rospy.loginfo("Image Resize paramters :")
        rospy.loginfo("top_x1 : %d, top_y1 : %d, bottom_x1 : %d, bottom_y1 : %d", config.top_x1, config.top_y1, config.bottom_x1, config.bottom_y1)
        rospy.loginfo("top_x2 : %d, top_y2 : %d, bottom_x2 : %d, bottom_y2 : %d", config.top_x2, config.top_y2, config.bottom_x2, config.bottom_y2)


        self.top_x1 = config.top_x1
        self.top_y1 = config.top_y1
        self.bottom_x1 = config.bottom_x1
        self.bottom_y1 = config.bottom_y1

        self.top_x2 = config.top_x2
        self.top_y2 = config.top_y2
        self.bottom_x2 = config.bottom_x2
        self.bottom_y2 = config.bottom_y2

        return config


    def cbImageProjection(self, msg_img):

        """
        converts the image and the draws line on the image coordinates, 
        these coordinates are used to crop the image.
        """

        #convert the image 
        cv_image_original = self.cvBridge.compressed_imgmsg_to_cv2(msg_img, "bgr8")

        top_x1 = self.top_x1
        top_y1 = self.top_y1
        bottom_x1 = self.bottom_x1
        bottom_y1 = self.bottom_y1


        top_x2 = self.top_x2
        top_y2 = self.top_y2
        bottom_x2 = self.bottom_x2
        bottom_y2 = self.bottom_y2

        # copy original image to use for drawing the lines.
        cv_image_calib = np.copy(cv_image_original)

        # draw lines to help find the required cropped image.
        cv_image_calib = cv2.line(cv_image_calib, ( top_x1,  top_y1), (top_x2, top_y2), (255,0,0), 1)
        cv_image_calib = cv2.line(cv_image_calib, ( bottom_x1,  bottom_y1), ( bottom_x2,bottom_y2), (0,255,0), 1)
        cv_image_calib = cv2.line(cv_image_calib, (bottom_x1,  bottom_y1), (top_x1,top_y1), (0, 0, 255), 1)
        cv_image_calib = cv2.line(cv_image_calib, (bottom_x2,  bottom_y2), (top_x2,top_y2), (255, 255, 255), 1)
        # publish the resulting image.
        self.pub_image_calib.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_calib, "jpg"))


    def main(self):
        rospy.spin()    

if __name__ == '__main__':
    rospy.init_node('image_projection')
    node = ImageProjection()
    node.main()
