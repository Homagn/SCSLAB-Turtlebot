#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
import math
from random import randint
import time as t
from datetime import datetime as dt
import threading
import os


class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class mover:
    def __init__(self,freq):
        #self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=100)
        rospy.init_node('wander')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(freq)
        self.x_real,self.x_sim = 0.0,0.0
        self.y_real,self.y_sim = 0.0,0.0
        self.r_real,self.r_sim = 0.0,0.0
        self.z_real,self.z_sim = 0.0,0.0
        self.blockListDict = {'block_a': Block('mobile_base', 'ground_plane_0')}
        self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.wait_for_service('/gazebo/set_model_state')
        t.sleep(1)
    def odom_callback(self,msg):
        #print(type(msg.pose.pose))
        self.x_real = msg.pose.pose.position.x
        self.y_real = msg.pose.pose.position.y
        self.z_real = msg.pose.pose.position.z
        self.r_real = msg.pose.pose.orientation.z
    def move(self,x,y,rotate,time): #blind move for real robot
        self.twist = Twist()
        self.twist.linear.x = x
        self.twist.linear.y = y
        self.twist.angular.z = rotate
        for i in range(time):
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
    def move_to_pos(self): #continuous control to move to a specified position 
        #Take feedback from self.x_real, self.y_real, etc
        #To implement
        return
    def get_model_pos(self): #get model position in simulator
        for block in self.blockListDict.itervalues():
            blockName = str(block._name)
            resp_coordinates = self.model_coordinates(blockName, block._relative_entity_name)
            print '\n'
            print 'Status.success = ', resp_coordinates.success
            print(blockName)
            #print("Cube " + str(block._name))
            self.x_sim = str(resp_coordinates.pose.position.x)
            self.y_sim = str(resp_coordinates.pose.position.y)
            self.z_sim = str(resp_coordinates.pose.position.z)
            self.r_sim = str(resp_coordinates.pose.orientation.z)

            print("value in x : " + str(resp_coordinates.pose.position.x))
            print("value in y : " + str(resp_coordinates.pose.position.y))
            print("value in r : " + str(resp_coordinates.pose.orientation.z))
            #print("Quaternion X : " + str(resp_coordinates.pose.orientation.x))
    def teleport(self, x, y, r): #move extremely fast (like a discreet step)
        state_msg = ModelState()
        state_msg.model_name = 'mobile_base'
        state_msg.pose.position.x = float(self.x_sim) + x
        state_msg.pose.position.y = float(self.y_sim) + y
        state_msg.pose.position.z = float(self.z_sim) + 0.0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = float(self.r_sim) + r
        state_msg.pose.orientation.w = 0
        #rospy.wait_for_service('/gazebo/set_model_state') #already called in init
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
if __name__ == '__main__':
    m=mover(1000)
    #t1 = threading.Thread(target=m.move, args=(5,-1,0.1,20))
    #t1.start()
    #t1.join()
    print(dt.now())
    for i in range(20):
        m.get_model_pos()
        m.teleport(-0.5,0.5,0.1)
        print(dt.now())
    