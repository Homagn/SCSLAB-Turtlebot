#This is a basic occupancy slam code
import numpy as np
import sys
import rospy
#print(sys.path)
from Helpers.camera import Camera
from Helpers.laser_scan import lscan
from Helpers.motion import mover
from Helpers.pointcloud import pointcloud
import cv2
import time as t
import random
import copy
import math

class slam(object):
    def __init__(self):
        rospy.init_node('env_observers') #This is the common node for camera, lidar, movement
        self.persistent_map = np.zeros((25,25)) #A 100x100 global map that stores either 0 
        #if the place can be visited (not blocked) or 1 if the place cannot be visited (or blocked)
        self.egocentric_map_t = np.zeros((25,25)) #Have an odd number  

        self.c = Camera("raw")
        self.l = lscan()
        self.m = mover(1000)
        self.p = pointcloud(viz_save = True)
        self.m.get_model_pos()
        self.state = [float(self.m.x_sim),float(self.m.y_sim),float(self.m.r_sim)]

        self.rx = float(self.m.x_sim) 
        self.ry = float(self.m.y_sim) 
        self.rot = float(self.m.r_sim)

        self.x_past = float(self.m.x_sim) 
        self.y_past = float(self.m.y_sim) 
        self.r_past = float(self.m.r_sim)

        self.project_scale = 10.0
        self.pcd_clipx = [-1.5,1.5]
        self.pcd_clipz = [0.69, 2.0]

        X = int(self.project_scale*(self.pcd_clipx[1]-self.pcd_clipx[0])) 
        Y = int(self.project_scale*(self.pcd_clipz[1])) +1
        self.gmap_pos = [X,int(Y/2)]
    def store_position(self):
        self.x_past = float(self.m.x_sim)
        self.y_past = float(self.m.y_sim)
        #self.r_past = float(self.m.r_sim)
    def resolve(self,a,b):
        if(a>b):
            return "left"
        else:
            return "right"
    def crop_pcd(self,pcd,cam):
        #This function is needed to resolve scaling issues with depth map
        #imagine the agent being close and then far away from the same wall
        #We make sure that only depth values between -1 and 1 are registered in the depth map
        #Another way to do this would be to normalize the whole pcd matrix between -1 and 1(?)
        print("max of pcd ",np.max(pcd[:,:,0]))
        print("min of pcd ",np.min(pcd[:,:,0]))

        print("max of pcd ",np.max(pcd[:,:,2]))
        print("min of pcd ",np.min(pcd[:,:,2]))
        for i in range(pcd.shape[0]):
            for j in range(pcd.shape[1]):
                k=0 #depth_x
                if(pcd[i,j,k]<self.pcd_clipx[0] or pcd[i,j,k]>self.pcd_clipx[1]):
                    pcd[i,j,k]=1000.0
                k=2 #depth_z
                if(pcd[i,j,k]>self.pcd_clipz[1]): #depth_z starts from the value 0.69
                    pcd[i,j,k]=1000.0
                if(cam[i,j]==0): #dont register depth for which image shows uncollidable
                    pcd[i,j,k]=1000.0
        return pcd
    def ground_project(self):
        cam = self.detect_blocks(self.c.see()) #This image should contain 1 where 
        #there is anything and 0 where there is empty space in the image
        cam = np.asarray(cam,dtype = np.float32)
        print(cam[0,0])
        pcd = self.p.see()
        #crop the pcd based on the set bounds on depth
        pcd = self.crop_pcd(pcd,cam)

        competing_elements = []
        competing_elements_pixel = []
        for i in range (480): #image_dim_x
            for j in range (640): #image_dim_y
                element = [int(self.project_scale*pcd[i,j,0]),int(self.project_scale*pcd[i,j,2])]
                if element[0]>=1000 or element[1]>=1000: #These are mostly garbage/NAN mappings
                    continue
                if(element in competing_elements):
                    #resolve function here !
                    #Dont need here because we are using perfect thresholding
                    continue
                else:
                    pixel = cam[i,j]
                    competing_elements.append(element)
                    competing_elements_pixel.append(pixel)

        m1=[]
        m2=[]
        if(len(competing_elements)==0):
            print("In a vast open space !")
            X = int(self.project_scale*(self.pcd_clipx[1]-self.pcd_clipx[0])) +1
            sX = int(X/2)+min(m1)
            Y = int(self.project_scale*(self.pcd_clipz[1])) +1
            self.egocentric_map_t = np.zeros((X,Y))
        else:
            for i in competing_elements:
                m1.append(i[0])
                m2.append(i[1])
            print("max of competing elements ",max(m1))
            print("max of competing elements ",max(m2))
            print("min of competing elements ",min(m1))
            print("min of competing elements ",min(m2))

            X = int(self.project_scale*(self.pcd_clipx[1]-self.pcd_clipx[0])) +1
            sX = int(X/2)+min(m1)
            Y = int(self.project_scale*(self.pcd_clipz[1])) +1
            self.egocentric_map_t = np.zeros((X,Y))
            print("resolving and constructing local map")
            for i in range(len(competing_elements)):
                #In order to follow numpy array ordering need to flip i[1] and i[0] positions
                #self.egocentric_map_t[competing_elements[i][1]+min(m2),competing_elements[i][0]+sX] = competing_elements_pixel[i]
                try:
                    self.egocentric_map_t[competing_elements[i][1],competing_elements[i][0]+sX] = competing_elements_pixel[i]
                except:
                    print("Error occured in ground projecting")
            self.egocentric_map_t = np.flip(self.egocentric_map_t,0)
        cv2.imwrite('ego_map.jpg', self.egocentric_map_t*255.0)
   
    def patch(self,source,target,targ_trans,targ_rot):
        #source is a np array
        #target is also (possibly smaller np array)
        targ_rot = math.radians(targ_rot)
        s = copy.copy(source)
        t = copy.copy(target)
        #s_c = [s.shape[0]-1,int(s.shape[1]/2)]
        #s_c = [s.shape[0]-1+source_trans[0],int(s.shape[1]/2)+source_trans[1]]
        s_c = [self.gmap_pos[0],self.gmap_pos[1]]
        s_maps = []
        s_maps_x = []
        s_maps_y = []
        s_v = []
        #translate source map to camera observer perspective
        for i in range(s.shape[0]):
            for j in range(s.shape[1]):
                s_maps.append([int(j-s_c[1]),int(s_c[0]-i)])
                s_maps_x.append(int(j-s_c[1]))
                s_maps_y.append(int(s_c[0]-i))
                s_v.append(s[i,j])
        #print("s_maps ",s_maps)
        #translate target map to camera observer perspective
        #plus apply the target translation too
        t_c = [t.shape[0]-1,int(t.shape[1]/2)]
        t_maps = []
        for i in range(t.shape[0]):
            for j in range(t.shape[1]):
                t_maps.append([int(j-t_c[1]+targ_trans[1]), int(t_c[0]-i+targ_trans[0]), t[i,j]])
        #Now rotate the target (anticlockwise)
        #print("t_maps ",t_maps)
        t_maps_r = []
        t_maps_x = []
        t_maps_y = []
        t_v = []
        for i in range(len(t_maps)):
            x = t_maps[i][0]*math.cos(targ_rot)-t_maps[i][1]*math.sin(targ_rot)
            y = t_maps[i][0]*math.sin(targ_rot)+t_maps[i][1]*math.cos(targ_rot)
            v = t_maps[i][2]
            t_maps_r.append([int(round(x)),int(round(y))])
            t_maps_x.append(int(round(x)))
            t_maps_y.append(int(round(y)))
            t_v.append(v)
        #print("t_maps_r ",t_maps_r)
        #Now patch together s_maps and t_maps_r
        X1 = max(max(t_maps_x),max(s_maps_x))  
        print("X1 ",X1)
        X2 = min(min(t_maps_x),min(s_maps_x))
        print("X2 ",X2)
        Y1 = max(max(t_maps_y),max(s_maps_y)) 
        print("Y1 ",Y1)
        Y2 = min(min(t_maps_y),min(s_maps_y))
        print("Y2 ",Y2)

        combined = np.zeros((Y1-Y2, X1-X2))
        for i in range(X2,X1):
            for j in range(Y2,Y1):
                val1 = 0.0
                val2 = 0.0
                val = 0.0
                num = 0
                if([i,j] in s_maps):
                    val1 = s_v[s_maps.index([i,j])]
                    #print([i,j])
                    num+=1
                if([i,j] in t_maps_r):
                    val2 = t_v[t_maps_r.index([i,j])]
                    #print([i,j])
                    num+=1
                if num!=0:
                    val = (float(val1)+float(val2))/num
                combined[j-Y2,i-X2] = val
        combined = np.flip(combined,0)
        #Update position in global map
        self.gmap_pos[0] = self.gmap_pos[0] + targ_trans[0] #- X2
        self.gmap_pos[1] = self.gmap_pos[1] + targ_trans[1] #- Y2

        return combined


    def localize(self):
        #convolve self.egocentric_map_t over self.persistent_map and estimate the most probable location and orientation
        #Or use dense matching/template matching
        self.m.get_model_pos()
        self.rx = self.project_scale*(float(self.m.x_sim) - self.x_past)
        self.ry = self.project_scale*(float(self.m.y_sim) - self.y_past)
        self.rot = float(self.m.r_sim) #- self.r_past
        #We are using perfect feedback from Gazebo
        #Otherwise need a Kalman Filter approach combining dense matching with robot kinematics 
        print("perceived relative movement ",self.rx,self.ry, math.degrees(self.rot-self.r_past))
        return self.rx,self.ry,math.degrees(self.rot-self.r_past)  

    def register_update(self,x,y,r):
        try:
            a = cv2.imread('persistent_map.jpg',0)
            b = cv2.imread('ego_map.jpg',0)
            c = s.patch(a,b,[-x,-y],r) #pass -y due to the way numpy array orders
            cv2.imwrite('persistent_map.jpg', c)
        except:
            print("persistent map does not exist, maybe first try")
            b = cv2.imread('ego_map.jpg',0)
            cv2.imwrite('persistent_map.jpg', b)
        
    def random_movement(self):
        print("Turtlebot moving randomly ")
        self.m.move(random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),int(random.uniform(0,1)*10.0))
    def detect_blocks(self,image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('rgb_image.jpg', image)

        get_edge = cv2.Canny(image, 10, 100)
        canny = np.hstack([get_edge])
        cv2.imwrite('canny_image.jpg', canny)

        blurred = cv2.GaussianBlur(image, (3, 3), 0)
        #Detect the ground plane
        cv_image = cv2.threshold(blurred, 30, 40, cv2.THRESH_BINARY)[1]
        #cv_image2 = cv2.threshold(blurred, 25, 30, cv2.THRESH_BINARY)[1]
        #cv_image = 0.5*(cv_image1+cv_image2)
        '''
        #thresholding inversion
        for i in range(cv_image.shape[0]):
            for j in range(cv_image.shape[1]):
                if(cv_image[i,j]==0.0):
                    cv_image[i,j]=1.0
                else:
                    cv_image[i,j]=0.0
        '''
        cv2.imshow("blocks", cv_image)
        cv2.waitKey(1)
        cv2.imwrite('thresh.jpg', cv_image*255.0)
        #print("Thresholded image ",cv_image)
        return np.array(cv_image).reshape((480,640))

                
if __name__ =='__main__':
    
    s = slam()

    while(True):
        testVar = raw_input("Finished moving? (y/n).")
        if(testVar=='y'):
            print("stopping SLAM")
            break
        x,y,r = s.localize()
        s.ground_project()
        
        s.register_update(x,y,r)
        s.store_position()
        #s.random_movement()
    
#Algorithm in brief:
'''
1. Initialize empty persistent map

2. Obtain egocentric map by fusing depth with image and ground projecting

3. Estimate relative difference in position (x,y) of the robot from initial (fixed) position in persistent 
    map from robot motion/kinematics

    Estimate relative difference in orientation R of the robot from initial position in persistent 
    map from robot motion/kinematics

4. Obtain something called relative map by cropping persistent map once relative displacement and
    relative rotation is known from 3
    Cropping rule:
    If robot is facing top-right diagonal (persistent map), crop the first quadrant by using 
    relative displacement positions
    If robot is facing top-left diagonal, crop the second quadrant by using relative displacement
    positions
    If robot is facing bottom-left diagonal, crop the third quadrant by using relative displacement
    positions
    If robot is facing bottom-right diagonal, crop the fourth quadrant by using relative displacement
    positions

    (obtaining relative map is extremely necessary in order to avoid false matching in step 5)
    False matching - the patches match but corresponding positions in real world dont match

5. Use template matching of egocentric map wrt relative map to localize the agent in the persistent
    map. (try to match various rotations of template with any position (A,B) in relative map and
    obtain a probability field- The index of the highest value in prob. field is the agent location)

6. Patch the egocentric map at the estimated location in relative map
    template re-rotation while patching:
    6.1 rotate the template (egocenric map) by R (step 3) while patching to relative map
    6.2 translate the template (egocenric map) by (A,B) (step 5) while patching to relative map

7. Patch the relative map to the persistent map. (update the cropped portion)
'''

#To do:
'''
1. Need perfect thresholding for detecting walls/obstacles in detect_blocks
2. Tests to do:
    static ground projection tests: close and away from a wall, near corner of 2 walls
    SLAM test: Moving towards a wall, away from wall, turning near a wall, turning near a corner
               moving along a long corridor (hallway)
3. ! something wrong with ego map creation- its not getting updated before persistent map
    Also the rotation angle supplied to patch function has to be made negetaive of current(prob)

3. s.random_movement() is very random, need an intelligent way to auto explore so that user dont have to do keyboard teleop
4. Use Neural networks for context based feature extraction from image
5. Use Neural networks for uncertainty removal from depth map/point_cloud (if any)
6. Use the persistent map concept for MADDPG
'''