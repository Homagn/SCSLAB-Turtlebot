#!/usr/bin/env python
'''
Generally people run rostopic subscriber and spin() (like an infinite while loop)
That means a extra terminal needs to always run camera code before running main code
That sucks!
Thats why this is a hacky code to access an instantaneous image from the camera.
No need to run seperately beforehand. Can be directly initialized as an obect
and call for the image whenever needed
'''
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from std_msgs.msg import String
import cv2, cv_bridge
import numpy
import time as t
import numpy as np
from datetime import datetime as dt
from collections import deque

class Camera:
  def __init__(self, imtype):
    self.bridge = cv_bridge.CvBridge()  
    self.image = []
    self.images = deque(maxlen=100)
    #Append a random something that is a np array and cant be read by cv2 as an image
    self.images.append(np.array(['a','b','c','d','e']))
    #self.images.append(np.array([1,2,3,4,5]))
    rospy.init_node('cam_observer')
    if imtype =="raw":
        #dont include subscribe in see fuction, it makes the code very slow (2-5x)
        rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
    if imtype =="compressed":
        rospy.Subscriber('camera/rgb/image_raw/compressed',CompressedImage, self.comp_image_callback)
  def see(self):
    while np.shape(self.images[-1])[0]!=3:
        try: 
            gray_image = cv2.cvtColor(self.images[-1], cv2.COLOR_BGR2GRAY)
            #self.res_img = cv2.resize(gray_image, (80, 80))
            #cv2.imshow("view", self.image)
            #cv2.waitKey(3)
            return self.images[-1]
        except:
            pass
  def see_compressed(self):
    #image_np = cv2.imdecode(self.image, cv2.CV_LOAD_IMAGE_COLOR) #in cv3
    while np.shape(self.images[-1])[0]!=3:
        try: 
            image_np = cv2.imdecode(self.images[-1], cv2.IMREAD_COLOR)
            #print(image_np)
            return image_np
        except:
            pass
    
  def comp_image_callback(self, msg):
    self.image = np.fromstring(msg.data, np.uint8)
    self.images.append(self.image)
  def image_callback(self, msg):
    self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    self.images.append(self.image)
    

#Testing functions 
if __name__ == '__main__':
    
    # Accessing raw image (should be slower but cant find evidence)
    #Average time to access an image - 0.002 seconds. Above 0.004 is too slow to run RL experiments
    c = Camera("raw")
    print(dt.now())
    for i in range(20):
        image = c.see()
        print(dt.now())
    cv2.imshow("view", image)
    #cv2.waitKey(30)
    status = cv2.imwrite('current_image.png',image) 
    print(np.shape(image))
    

    '''
    #Accessing compressed image - this one is apparently 6x slower!
    c = Camera("compressed")
    print(dt.now())
    for i in range(20):
        image = c.see_compressed()
        print(dt.now())
    print(np.shape(image))
    '''