#!/usr/bin/env python
import rospy
import struct
import ctypes
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import sys
import cv2
import numpy as np
import math
from collections import deque
from datetime import datetime as dt
import time as t
import ros_numpy #sudo apt-get install ros-kinetic-ros-numpy

#NOTE: It returns a 5 dimensional entity: At each 480x640 pixel locations it stores the laser distance x,y, and z from camera
class pointcloud:
  def __init__(self, viz_save = False): #Saves a visualization of the depth map (x,y,z independently) as a jpg image
    self.image = []
    self.images = deque(maxlen=100)
    #Append a random something that is a np array and cant be read by cv2 as an image
    self.images.append(np.array(['a','b','c','d','e']))
    self.scale = 1.0
    self.viz_save = viz_save
    #self.images.append(np.array([1,2,3,4,5]))
    #rospy.init_node('cam_observer')
    rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback2)
  def see(self):
    while np.shape(self.images[-1])[0]!=3:
        try: 
            #gray_image = cv2.cvtColor(self.images[-1]*200.0, cv2.COLOR_BGR2GRAY)
            r_image = self.images[-1].reshape((480,640,3))
            return self.images[-1]
        except:
            #print("OpenCV error")
            pass
  def callback2(self,msg): #5 times faster than callback1
    pc = ros_numpy.numpify(msg)
    height = pc.shape[0]
    width = pc.shape[1]
    np_points = np.zeros((height,width, 3), dtype=np.float32)
    np_points[:,:,0] = np.resize(pc['x'], (height,width))
    np_points[:,:,1] = np.resize(pc['y'], (height,width))
    np_points[:,:,2] = np.resize(pc['z'], (height,width))
    cv2.imwrite('x.jpg', np_points[:,:,0]*100)
    cv2.imwrite('y.jpg', np_points[:,:,1]*100)
    cv2.imwrite('z.jpg', np_points[:,:,2]*100)
    #cv2.imwrite('current_depth_image.png', np_points*200)
    print("File end")
    self.image = np_points
    self.images.append(self.image)

  def callback1(self,msg): #About 7 times faster than callback
    #data_out = pc2.read_points(msg, skip_nans=False) #Takes image the same size as camera - 480x640
    cloud_points = list(pc2.read_points(msg, skip_nans=False, field_names = ("x", "y", "z")))
    #print(cloud_points)
    x_part = [i[0] for i in cloud_points]
    y_part = [i[1] for i in cloud_points]
    z_part = [i[2] for i in cloud_points]

    scale = self.scale
    x_part = np.array(x_part).reshape((480,640))*scale
    y_part = np.array(y_part).reshape((480,640))*scale
    z_part = np.array(z_part).reshape((480,640))*scale
    #Remove NAN
    depth_msg = np.zeros((480,640,3))
    depth_msg[:,:,0]=x_part
    depth_msg[:,:,1]=y_part
    depth_msg[:,:,2]=z_part
    where_are_NaNs = np.isnan(depth_msg)
    depth_msg[where_are_NaNs] = 0.0

    cv2.imwrite('x.jpg', x_part)
    cv2.imwrite('y.jpg', y_part)
    cv2.imwrite('z.jpg', z_part)
    #status = cv2.imwrite('current_depth_image.png',depth_msg) 
    print("File end")
    
    self.image = depth_msg
    self.images.append(self.image)

  def callback(self, msg):
    #data_out = pc2.read_points(msg, skip_nans=True)
    data_out = pc2.read_points(msg, skip_nans=False) #Takes image the same size as camera - 480x640
    depth_msg = []
    x_part = []
    y_part = []
    z_part = []

    loop = True
    while loop:
        try:
            int_data = next(data_out)
            s = struct.pack('>f' ,int_data[3])
            i = struct.unpack('>l',s)[0]
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)

            #file.write(str(int_data[0])+","+str(int_data[1])+","+str(int_data[2])+","+str(r)+","+str(g)+","+str(b)+"\n")
            #depth_msg.append([int_data[0],int_data[0],int_data[0],r,g,b])
            depth_msg.append([int_data[0],int_data[1],int_data[2]])
            x_part.append([int_data[0]]) 
            y_part.append([int_data[1]]) 
            z_part.append([int_data[2]]) 
            #depth_msg.append([math.fabs(int_data[1])])
            #print(int_data[0]*480," ",int_data[1]*640," ",int_data[2])    

        except Exception as e:
            #rospy.loginfo(e.message)
            depth_msg = np.array(depth_msg).reshape((480,640,3))*self.scale
            where_are_NaNs = np.isnan(depth_msg)
            depth_msg[where_are_NaNs] = 0.0

            self.image = depth_msg
            self.images.append(self.image)
            if self.viz_save:
                print("File end")
                scale = 200.0
                x_part = np.array(x_part).reshape((480,640))*scale
                cv2.imwrite('x.jpg', x_part)

                y_part = np.array(y_part).reshape((480,640))*scale
                cv2.imwrite('y.jpg', y_part)

                z_part = np.array(z_part).reshape((480,640))*scale
                cv2.imwrite('z.jpg', z_part)
                status = cv2.imwrite('current_depth_image.png',self.image*100.0) 

            loop = False
            #file.flush
            #file.close
    

#Testing functions 
if __name__ == '__main__':

    #(callback2- fastest)
    rospy.init_node('depth_point_cloud')
    p = pointcloud(viz_save = True)
    t.sleep(1) #Need 1 second sleep for some reason
    print(dt.now())
    image = p.see()*200.0
    print(dt.now())
    status = cv2.imwrite('current_depth_image.png',image) 

    '''
    #(callback1)
    rospy.init_node('depth_point_cloud')
    p = pointcloud(viz_save = True)
    t.sleep(1) #Need 1 second sleep for some reason
    print(dt.now())
    image = p.see()*200.0
    print(dt.now())
    status = cv2.imwrite('current_depth_image.png',image) 
    '''

    #old function (callback)
    '''
    p = pointcloud(viz_save = True)
    #t.sleep(5)
    print(dt.now())
    image = p.see()*200.0
    print(dt.now())
    status = cv2.imwrite('current_depth_image.png',image) 
    print(np.shape(image))
    '''
    

