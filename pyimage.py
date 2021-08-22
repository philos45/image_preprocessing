#!/usr/bin/env python
# Capturing image from the color and depth ros topics of realsense camera
# Adapted from http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber("/zed2/zed_node/right/image_rect_color",Image,self.callback)
        #self.depth_sub = rospy.Subscriber("/realsense/camera/depth/image_raw",Image,self.callback)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            cv_image = self.bridge.imgmsg_to_cv2(data)
	    print(type(cv_image)) # numpy
        cv_image = cv_image[0:400,400:1800]
        # cv2.imshow('image', cv_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print(cv_image.shape) 

        pub = rospy.Publisher('receiver', Image, queue_size=10)
        rospy.init_node('image_converter', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = cv_image
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()



def main():
    ic = image_converter()
    # talker()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = image_converter()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    main()
