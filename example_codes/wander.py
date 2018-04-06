#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from random import randint


def scan_callback(msg):
  global g_range_ahead 
  #g_range_ahead = min(ranges)
  st=len(msg.ranges)/4
  ed=3*st
  md=2*st
  g_range_ahead=(msg.ranges[st]+msg.ranges[md]+msg.ranges[ed])/3
  if math.isnan(g_range_ahead):
	g_range_ahead=0.0
  print g_range_ahead

g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
  if driving_forward:
    if (g_range_ahead < 1.0 or rospy.Time.now() > state_change_time):
      driving_forward = False
      ran=randint(2,8)
      state_change_time = rospy.Time.now() + rospy.Duration(3)
  else: # we're not driving_forward
    if rospy.Time.now() > state_change_time:
      driving_forward = True # we're done spinning, time to go forward!
      state_change_time = rospy.Time.now() + rospy.Duration(30)
  twist = Twist()
  if driving_forward:
    twist.linear.x = 0.3
  else:
    twist.angular.z = 1.0
  cmd_vel_pub.publish(twist)

  rate.sleep()
