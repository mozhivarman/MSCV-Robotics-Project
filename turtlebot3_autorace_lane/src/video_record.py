#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge


mainVideo = cv2.VideoWriter('video/Main.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (640,480 ))
cropedVideo = cv2.VideoWriter('video/cropped.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (640,80 ))
YellowMaskVideo = cv2.VideoWriter('video/YellowMaskVideo.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (640,80 ))
WhiteMaskVideo = cv2.VideoWriter('video/WhiteMaskVideo.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (640,80 ))


def fnShutDown():
    mainVideo.release()
    cropedVideo.release()
    YellowMaskVideo.release()
    WhiteMaskVideo.release()


class rosCameraVideo:
    def __init__(self):
        # self.frame_width = None
        # self.frame_height = None
        # out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
        # out.release()

        self.Mainvideo = rospy.Subscriber('/raspicam_node/image/compressed',CompressedImage,callback=self.CallBackMain)
        self.cropedImage = rospy.Subscriber('/cropped_image/compressed',CompressedImage,callback=self.CallBackCrop)
        self.Yellowmask = rospy.Subscriber('/image/yellow/compressed',CompressedImage,callback=self.CallBackYellow)
        self.WhiteMask = rospy.Subscriber('/image/white/compressed',CompressedImage,callback=self.CallBackMainWhite)

        self.cvBridge = CvBridge()

    def CallBackMain(self,msg):
        image = self.cvBridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        if image is not None:        
            mainVideo.write(image)

    def CallBackCrop(self,msg):
        image = self.cvBridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        if image is not None:        
            cropedVideo.write(image)

    def CallBackYellow(self,msg):
        image = self.cvBridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        if image is not None:        
            image = cv2.merge([image,image,image])
            YellowMaskVideo.write(image)

    def CallBackMainWhite(self,msg):
        image = self.cvBridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        if image is not None:   
            image = cv2.merge([image,image,image])     
            WhiteMaskVideo.write(image)

if __name__ == '__main__':
    rospy.init_node('video_recored')
    
    try:
        vi = rosCameraVideo()
        rospy.on_shutdown(fnShutDown)
        rospy.spin()
    except rospy.ROSException() as e:
        print(e)