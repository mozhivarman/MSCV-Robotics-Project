#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
import sys


contrast   = rospy.get_param('contrast',20)
sharpness  = rospy.get_param('sharpness',100)
brightness = rospy.get_param('brightness',50)
saturation = rospy.get_param('saturation',25)
ISO        = rospy.get_param('ISO',100)
exposure_compensation = rospy.get_param('exposure_compensation',-4)


class Camrareconfigure : 
    def  __init__(self):
        self.client = dynamic_reconfigure.client.Client('/raspicam_node',20,config_callback=self.Callback)
        self.pa1 = None
        self.pa2 = None
        self.pa3 = None
        self.pa4 = None
        self.pa5 = None
        self.pa6 = None

    def Callback(self,config):
        print('contrast is: {contrast} '.format(**config))
        print('sharpness is: {sharpness} '.format(**config))
        print('brightness is: {brightness} '.format(**config))
        print('saturation is: {saturation} '.format(**config))
        print('ISO is: {ISO} '.format(**config))
        print('exposure_compensation is: {exposure_compensation} '.format(**config))
        self.pa1 = config['contrast'] == contrast 
        self.pa2 = config['sharpness'] == sharpness 
        self.pa3 = config['brightness'] == brightness  
        self.pa4 = config['saturation'] == saturation  
        self.pa5 = config['ISO'] == ISO  
        self.pa6 = config['exposure_compensation'] == exposure_compensation


    def main(self):
        self.client.update_configuration({'contrast' : contrast,
                                 'sharpness' : sharpness,
                                 'brightness' : brightness,
                                 'saturation': saturation,
                                 'ISO' : ISO,
                                 'exposure_compensation' : exposure_compensation})
        if self.pa1 & self.pa2 & self.pa3 & self.pa4 & self.pa5 & self.pa6 :
            print("All Parameters are set")
            sys.exit()
        else : rospy.spin()



if __name__ == "__main__":
    rospy.init_node('reconfigure_camera')
    # rospy.set_param('/raspicam_node/contrast',20)
    # rospy.set_param('/raspicam_node/sharpness',100)
    # rospy.set_param('/raspicam_node/brightness',50)
    # rospy.set_param('/raspicam_node/saturation',25)
    # rospy.set_param('/raspicam_node/ISO',100)
    # rospy.set_param('/raspicam_node/exposure_compensation',-4)

    camera = Camrareconfigure()
    try: 
        camera.main()
    except rospy.ROSException as e:
        print(e)
        
  



