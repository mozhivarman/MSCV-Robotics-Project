#!/usr/bin/env python

#####
# The code below computes the moment for the filterd image and calculates the 
# centroid of the yellow and the white lines, so that the robot is in the middle of the line.
#####




from sys import path
import cv2
import rospy
import numpy as np
import rospkg
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image,CompressedImage



class LineMoment:
    def __init__(self,Path):

        self.top_x1 = rospy.get_param("/top_x11", 0)
        self.top_y1 =  rospy.get_param("/top_y1", 415)
        self.bottom_x1 = rospy.get_param("/bottom_x1", 640)
        self.bottom_y1 =     rospy.get_param("/bottom_y1", 415)
        
        self.top_x2 = rospy.get_param("/top_x2", 0)
        self.top_y2 =  rospy.get_param("/top_y2", 480)
        self.bottom_x2 = rospy.get_param("/bottom_x2", 640)
        self.bottom_y2 =     rospy.get_param("/bottom_y2", 480)


        
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed',CompressedImage,callback=self.CallBack)
        self.pub = rospy.Publisher('/cropped_image/compressed',CompressedImage,queue_size=1)
        self.pub_yellow = rospy.Publisher('/image/yellow/compressed',CompressedImage,queue_size=1)
        self.pub_white = rospy.Publisher('/image/white/compressed',CompressedImage,queue_size=1)
        self.control_lane = rospy.Publisher('/control_lane', Float64, queue_size = 1)

        self.pathTOYellowMask = Path+'/config/mask_yellow.png'
        self.pathTOWhiteMask  = Path+'/config/mask_white.png'

        self.lastYellowMask = cv2.imread(self.pathTOYellowMask,0)
        self.lastWhiteMask =  cv2.imread(self.pathTOWhiteMask,0)

        if self.lastWhiteMask is None:
            rospy.logerr("lastWhiteMask is None")
        if self.lastYellowMask is None:
            rospy.logerr("lastWhiteMask is None")

        self.HSVYellowmaskMin = rospy.get_param('YellowMaskMinHSV',[0,100,170])
        self.HSVYellowmaskMax = rospy.get_param('YellowMaskMaxHSV',[90,255,255])

        self.HSVWthiemaskMin = rospy.get_param('WhiteMaskMinHSV',[30,0,173])
        self.HSVWhitemaskMax = rospy.get_param('WHiteMaskMaxHSV',[90,65,255])

        self.control_lane_msg = Float64()


        self.cvBridge = CvBridge()

        self.printImageProjectionParam()
        
    def printImageProjectionParam(self):
        rospy.loginfo("Image Resize paramters :")
        rospy.loginfo("top_x1 : %d, top_y1 : %d, bottom_x1 : %d, bottom_y1 : %d", self.top_x1, self.top_y1, self.bottom_x1, self.bottom_y1)
        rospy.loginfo("top_x2 : %d, top_y2 : %d, bottom_x2 : %d, bottom_y2 : %d", self.top_x2, self.top_y2, self.bottom_x2, self.bottom_y2)
   
    def CallBack(self,msg):
        image = self.cvBridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        # image = image[415:480,0:635]
        image = image[self.bottom_y1-15:self.bottom_y2,self.top_x1:self.bottom_x1]
        height,width = image.shape[0:2]
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)   
        # print(image.shape)     

        mask_yellow = cv2.inRange(hsv,tuple(self.HSVYellowmaskMin),tuple(self.HSVYellowmaskMax))
        mask_white = cv2.inRange(hsv,tuple(self.HSVWthiemaskMin),tuple(self.HSVWhitemaskMax))


        # cv2.imwrite('mask_white.png',mask_white)
        # cv2.imwrite('mask_yellow.png',mask_yellow)
        if  np.count_nonzero(mask_yellow) < 35:
            mask_yellow  = self.lastYellowMask

        m = cv2.moments(mask_yellow, False )
        try:
            cx1, cy1 = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy1, cx1 = height/2, width/2
        cv2.circle(image,(int(cx1), int(cy1)), 15,(0,0,255),-1)

        if  np.count_nonzero(mask_white) < 35:
            mask_white  = self.lastWhiteMask
        
        m = cv2.moments(mask_white, False)
        try:
            cx2, cy2 = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy2, cx2 = height/2, width/2    
        cv2.circle(image,(int(cx2), int(cy2)), 15,(0,255,0),-1)

        cx3,cy3 = int((cx1+cx2)/2), int((cy1+cy2)/2)

        cv2.circle(image,(int(cx3), int(cy3)), 15,(255,0,0),-1)

        self.pub.publish(self.cvBridge.cv2_to_compressed_imgmsg(image, "jpg"))

        self.pub_yellow.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask_yellow, "jpg"))
        self.pub_white.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask_white, "jpg"))

        self.control_lane_msg.data = float(cx3)
        self.control_lane.publish(self.control_lane_msg)
        print( "The centroid of the image : {}".format(self.control_lane_msg.data ))

if __name__ == '__main__':
    # Initialize a ROS Node
    rospy.init_node('line_moment')
    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('turtlebot3_autorace_lane')
    line = LineMoment(packagePath)
    rospy.wait_for_message('/raspicam_node/image/compressed',CompressedImage,timeout=10)
    
    try:
        rospy.spin()
    except rospy.ROSException as e:
        print(e)
