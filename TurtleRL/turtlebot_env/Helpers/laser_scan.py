#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from random import randint
from collections import deque
import numpy as np
import time as t
from datetime import datetime as dt

class lscan:
  def __init__(self):
    self.scan =[]
    self.scans = deque(maxlen=10)
    #self.scans.append(['a'])
    rospy.init_node('scan_observer')
    rospy.Subscriber('/scan', LaserScan, self.scan_callback)
    t.sleep(1) #1 second initialization time to start getting scans
  def see(self):
    a = self.scans[-1]
    b=[]
    for i in a:
        if np.isnan(i):
            b.append(-1.0)
        else:
            b.append(i)
    return b

  def scan_callback(self, msg):
    self.scan = msg.ranges
    self.scans.append(self.scan)
    

#Testing functions 
if __name__ == '__main__':
    l = lscan()
    print(dt.now())
    for i in range(20):
        ls = l.see()
        print(np.array(ls))
        print(dt.now())
    print(np.array(ls))
    