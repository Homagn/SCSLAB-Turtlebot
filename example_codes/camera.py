#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2, cv_bridge
import numpy



class Follower:

   
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    
    
    
    
    
  
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    bt=float(0.0)
    btt=float(0.0)
    gt=float(0.0)
    gtt=float(0.0)
    rt=float(0.0)
    rtt=float(0.0)
    for i in xrange(0, 479, 10):
	
	bm=float(0.0)
	gm=float(0.0)
	rm=float(0.0)
	for j in xrange(0,639,10):
		k=float(((240-i)*(240-i)))
		blue=float(image[i,j,0])
		green=float(image[i,j,1])
		red=float(image[i,j,2])
		bm=bm+blue
		gm=gm+green
		rm=rm+red
		
	bt=bt+bm*k
	btt=btt+bm
	gt=gt+gm*k
	gtt=gtt+gm
	rt=rt+rm*k
	rtt=rtt+rm
    valib=float(bt/btt)
    valig=float(gt/gtt)
    valir=float(rt/rtt)
   
    
    bt1=float(0.0)
    btt1=float(0.0)
    gt1=float(0.0)
    gtt1=float(0.0)
    rt1=float(0.0)
    rtt1=float(0.0)
    for s in xrange(0, 639, 10):
	
	bm1=float(0.0)
	gm1=float(0.0)
	rm1=float(0.0)
	for t in xrange(0,479,10):
		k1=float(((320-s)*(320-s)))
		blue=float(image[t,s,0])
		green=float(image[t,s,1])
		red=float(image[t,s,2])
		bm1=bm1+blue
		gm1=gm1+green
		rm1=rm1+red
	
	bt1=bt1+bm1*k1
	btt1=btt1+bm1
	gt1=gt1+gm1*k1
	gtt1=gtt1+gm1
	rt1=rt1+rm1*k1
	rtt1=rtt1+rm1
    valjb=float(bt1/btt1)
    valjg=float(gt1/gtt1)
    valjr=float(rt1/rtt1)
    cv2.imshow("window", image)
    valib=int(valib/100)
    valjb=int(valjb/100)
    out=valib*1000+valjb
    r_out  = repr(out)
    print valib
    print valjb
    #mesg=repr(out)%rospy.get_time()
    mesg = (r_out + "..." +"%s") %rospy.get_time()
    rospy.loginfo(mesg)
    pub.publish(mesg)
    rate.sleep()
    cv2.waitKey(3)

pub = rospy.Publisher('chatter', String, queue_size=10) 
rospy.init_node('follower')
rate = rospy.Rate(10)
follower = Follower()
rospy.spin()
