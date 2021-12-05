#!/usr/bin/env python
# -*- coding: utf-8 -*



######
# The code below is used to get the HSV Values of the a 
# particular color in the HSV space.Uses Opencv's function 
# to filter the color and display the image.
######

##import the required packages
import cv2
import rospy
### To get the image from the published image
from ImageSubscriber import imageSubsribe


def callback(value):
    pass


def setup_trackbars(range_filter):
    """
    setting the up the trackbar for filtering with OPENCV.
    """
    ##
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)

def get_trackbar_values(range_filter):
    """
    setting Value for the trackbar.
    """
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


# main function

def main():
    """
    subscribes to a image topic and apply the filter according to the 
    values that are returned from the trackbar.
    """
    # initlizing the rosNode.
    rospy.init_node("hsv_detector", log_level=rospy.DEBUG)
    rospy.logwarn("Starting....")

    #initlizing to get Image from the image topic.
    sensors_obj = imageSubsribe('/raspicam_node/image/compressed')
    # Image returned in opencv fromat.
    cv_image = sensors_obj.get_image()

    # if No image is returned then sleep for 5  seconds and continue.
    if cv_image == None:
        rospy.sleep(5)

    range_filter = "HSV"

    # initlizing the track bar to get the HSV values for filtering.
    setup_trackbars(range_filter)

    while not rospy.is_shutdown():
        #image from the publisher.
        image = sensors_obj.get_image()
        #convert image to HSV color space from BGR color space.
        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #getting filtering values from trackbar.
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        # filtering the with values to get the mask.
        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        #applying the mask to get the filtered image.
        preview = cv2.bitwise_and(image, image, mask=thresh)

        cv2.imshow("Preview", preview)
        # press 'q' to exit the program.
        if cv2.waitKey(1) & 0xFF is ord('q'):
            break

    rospy.logwarn("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()