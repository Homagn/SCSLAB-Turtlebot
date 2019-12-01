#This is a basic occupancy slam code
import numpy as np
import sys

from client_correct import gazebo_client
'''
#only needed when using locally with ROS installed 
import rospy
#print(sys.path)
from Helpers.camera import Camera
from Helpers.laser_scan import lscan
from Helpers.motion import mover
from Helpers.pointcloud import pointcloud
'''

import cv2
import time as t
import random
import copy
import math
from datetime import datetime as dt

class slam(object):
    def __init__(self):
        self.gc = gazebo_client() #make sure (main_server.py) is running on remote compter hosting gazebo
        #self.gc.step('0.0,0.0,0.0') #intialize the robot to origin

        #rospy.init_node('env_observers') #This is the common node for camera, lidar, movement

        self.persistent_map = np.zeros((25,25)) #A 100x100 global map that stores either 0 
        #if the place can be visited (not blocked) or 1 if the place cannot be visited (or blocked)
        self.egocentric_map_t = np.zeros((25,25)) #Have an odd number  

        self.x_sim = 0.0
        self.y_sim = 0.0
        self.r_sim = 0.0
        '''
        self.c = Camera("raw")
        self.l = lscan()
        self.m = mover(1000)
        self.p = pointcloud(viz_save = True)
        t.sleep(1)
        '''

        #self.m.get_model_pos()

        self.state = [float(self.x_sim),float(self.y_sim),float(self.r_sim)]

        self.rx = float(self.x_sim) 
        self.ry = float(self.y_sim) 
        self.rot = float(self.r_sim)

        self.x_past = 0.0 
        self.y_past = -0.3
        self.r_past = 1.57

        
        self.pcd_clipx = [-1.5,1.5]
        self.pcd_clipz = [0.69, 2.0]
        #magic formula to maintain scale invariance while ground projecting
        self.bz = math.tan(self.pcd_clipz[0]/1024 + 0.5)*33.825 + 5.7 # 24.20828
        self.bx = math.tan(self.pcd_clipx[0]/1024 + 0.5)*33.825 + 5.7 # 24.11439

        self.mz = math.tan(self.pcd_clipz[1]/1024 + 0.5)*33.825 + 5.7 # 24.20828
        self.mx = math.tan(self.pcd_clipx[1]/1024 + 0.5)*33.825 + 5.7 # 24.11439

        self.project_scale = 1000.0 
        self.pix_move_scale_y = 43.181 #Value found out by experiments for this pcd clip and project_scale
        self.pix_move_scale_x = 42.904 #Value found out by experiments for this pcd clip and project_scale
        #self.pix_move_scale_x = 51.42 #Value found out by experiments for this pcd clip and project_scale
        #X = int(self.project_scale*(self.pcd_clipx[1]-self.pcd_clipx[0])) 
        #Y = int(self.project_scale*(self.pcd_clipz[1])) +1

        X = int(round((self.mx-self.bx)*self.project_scale))+1
        Y = int(round((self.mz-self.bz)*self.project_scale))+1

        self.egocentric_map_t = np.zeros((Y,X))
        self.offset = [-Y,X/2]
        self.offs = [Y,int(X/2)]

        self.map_pos = [Y,int(X/2)]
    def initial_localization(self):
        #invoke if a persistent map already exists
        #at each time step rotate a small amount and create self.egocentric_map_t
        #convolve self.egocentric_map_t over self.persistent_map and estimate the most probable location
        #repeat above two steps by rotating turtlebot in slow steps to 360 each time
        #Thus find the most probable location and orientation in the already existing map
        #Or use dense matching/template matching
        return
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
        #print("max of pcd ",np.max(pcd[:,:,0]))
        #rint("min of pcd ",np.min(pcd[:,:,0]))

        #print("max of pcd ",np.max(pcd[:,:,2]))
        #print("min of pcd ",np.min(pcd[:,:,2]))
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
        #cam = self.detect_blocks(self.c.see()) #This image should contain 1 where 
        cam = self.detect_blocks(self.cam)
        #there is anything and 0 where there is empty space in the image
        cam = np.asarray(cam,dtype = np.float32)
        print("testing time ",dt.now())
        #pcd = self.p.see()
        #baseline = 100*np.ones((480,640,3))
        #pcd = cv2.imread('data/depth_image.png') - baseline #image was multipled by 200.0 and saved.
        pcd = self.depth
        print("testing time ",dt.now())
        #crop the pcd based on the set bounds on depth
        pcd = self.crop_pcd(pcd,cam)
        print("testing time ",dt.now())

        X = int(round((self.mx-self.bx)*self.project_scale))+1
        Y = int(round((self.mz-self.bz)*self.project_scale))+1

        self.egocentric_map_t = np.zeros((Y,X))

        competing_elements = []
        competing_elements_pixel = []
        for i in range (480): #image_dim_x
            for j in range (640): #image_dim_y
                e1 = int(round(self.project_scale*((math.tan(pcd[i,j,0]/1024 + 0.5)*33.825 + 5.7)-self.bx)))
                e2 = int(round(self.project_scale*((math.tan(pcd[i,j,2]/1024 + 0.5)*33.825 + 5.7)-self.bz)))
                #element = [int(self.project_scale*pcd[i,j,0]),int(self.project_scale*pcd[i,j,2])]
                element = [e1,e2]
                if pcd[i,j,0]>=1000 or pcd[i,j,2]>=1000: #These are mostly garbage/NAN mappings
                    continue
                if(element in competing_elements):
                    #resolve function here !
                    #Dont need here because we are using perfect thresholding
                    continue
                else:
                    pixel = cam[i,j]
                    competing_elements.append(element)
                    competing_elements_pixel.append(pixel)
        print("testing time ",dt.now())
        m1=[]
        m2=[]
        #print("competing elements ",competing_elements)
        if(len(competing_elements)==0):
            print("In a vast open space !")
            
        else:
            for i in competing_elements:
                m1.append(i[0])
                m2.append(i[1])
            #print("max of competing elements ",max(m1))
            #print("max of competing elements ",max(m2))
            #print("min of competing elements ",min(m1))
            #print("min of competing elements ",min(m2))

            #X = int(self.project_scale*(self.pcd_clipx[1]-self.pcd_clipx[0])) +1
            #Y = int(self.project_scale*(self.pcd_clipz[1])) +1
            #self.egocentric_map_t = np.zeros((X,Y))
            print("resolving and constructing local map")
            for i in range(len(competing_elements)):
                #In order to follow numpy array ordering need to flip i[1] and i[0] positions
                #self.egocentric_map_t[competing_elements[i][1]+min(m2),competing_elements[i][0]+sX] = competing_elements_pixel[i]
                self.egocentric_map_t[competing_elements[i][1],competing_elements[i][0]] = competing_elements_pixel[i]
                try:
                    self.egocentric_map_t[competing_elements[i][1],competing_elements[i][0]] = competing_elements_pixel[i]
                except:
                    print("Error occured in ground projecting")
            self.egocentric_map_t = np.flip(self.egocentric_map_t,0)
        cv2.imwrite('ego_map.jpg', self.egocentric_map_t*255.0)

    def rotateImage(self,image,angle):
        image_center = tuple(np.array(image.shape[1::-1])/2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result
    def patch2(self,source,target,targ_trans,testing=False):
        s = np.array(copy.copy(source))
        global_map = copy.copy(s)

        s_end_x = s.shape[0]
        s_end_y = s.shape[1]

        t = np.array(copy.copy(target))
        tb = [t.shape[0]-1, int(t.shape[1]/2)]
        if testing:
            sb = [(s.shape[0]-1),s.shape[1]/2]
        else:
            sb = [self.map_pos[0], self.map_pos[1]]
            #self.map_pos[0] = self.offs[0] #- targ_trans[0]
            #self.map_pos[1] = self.offs[1] #+ targ_trans[1]
        #x is downwards, y is sideways
        patch_start_x = sb[0]-tb[0]+targ_trans[0]
        patch_start_y = sb[1]-tb[1]+targ_trans[1]

        patch_end_x = sb[0]+targ_trans[0]+1
        patch_end_y = sb[1]+tb[1]+targ_trans[1]


        while patch_start_x<0:
            #pad arguments go for top,down.left,right accordingly
            global_map = np.pad(global_map, ((1, 0),(0,0)), 'constant',constant_values=0)
            patch_start_x+=1
            patch_end_x+=1
            self.map_pos[0]+=1

        while patch_start_y<0:
            global_map = np.pad(global_map, ((0, 0),(1,0)), 'constant',constant_values=0)
            patch_start_y+=1
            patch_end_y+=1
            self.map_pos[1]+=1

        while patch_end_x>s_end_x:
            global_map = np.pad(global_map, ((0, 1),(0,0)), 'constant',constant_values=0)
            s_end_x+=1

        while patch_end_y>s_end_y:
            global_map = np.pad(global_map, ((0, 0),(0,1)), 'constant',constant_values=0)
            s_end_y+=1

        global_map[patch_start_x:patch_end_x, patch_start_y:patch_end_y] += t
        return global_map

    def send_motion(self,moves):
        self.x_sim = moves[0]
        self.y_sim = moves[1]
        self.r_sim = moves[2]
        move_str = repr(moves[0])+','+repr(moves[1])+','+repr(moves[2])
        self.cam,self.depth,self.pos = self.gc.step(move_str)
        print("got pos from server ",self.pos)
        self.x_sim = self.pos[0]
        self.y_sim = self.pos[1]
        self.r_sim = self.pos[2]
    def store_motion(self):
        self.x_past = self.pos[0]
        self.y_past = self.pos[1]
        self.r_past = self.pos[2]
    def localize(self):
        #convolve self.egocentric_map_t over self.persistent_map and estimate the most probable location and orientation
        #Or use dense matching/template matching
        #self.m.get_model_pos()
        #self.rx = self.project_scale*(float(self.m.x_sim) - self.x_past)
        #self.ry = self.project_scale*(float(self.m.y_sim) - self.y_past)
        self.rx = float(self.x_sim) - self.x_past
        self.ry = float(self.y_sim) - self.y_past
        self.rot = float(self.r_sim) #- self.r_past
        #We are using perfect feedback from Gazebo
        #Otherwise need a Kalman Filter approach combining dense matching with robot kinematics 
        print("perceived relative movement ",self.rx,self.ry, math.degrees(self.rot-self.r_past))
        return self.rx,self.ry,math.degrees(self.rot-self.r_past)  

    def register_update(self,x,y,r):
        try:
            a = cv2.imread('persistent_map.jpg',0)
            b = cv2.imread('ego_map.jpg',0)
            
            c = self.rotateImage(b,round(r))
            cv2.imwrite('rotated.jpg', c)

            rad_r = math.radians(round(r))
            offset_x = 0
            offset_y = 0
            if math.floor(math.fabs(r))!=0:
                offset_y = (1-math.cos(rad_r))*(-self.offset[0])
                #offset_x = (1+math.sin(rad_r))*(self.offset[1])
                offset_x = (-math.sin(rad_r))*(self.offset[1])
            Y = y*self.pix_move_scale_y - offset_y
            X = x*self.pix_move_scale_x + offset_x
            print(y*self.pix_move_scale_y)
            print(x*self.pix_move_scale_x)
            print(Y)
            print(X)

            d = self.patch2(a,c,[-int(Y),int(X)])
            cv2.imwrite('persistent_map.jpg', d)
            
            '''
            #Tests used for finding self.pix_move_scale_y and self.pix_move_scale_x
            c = s.patch2(a,b,[0,0]) #pass -y due to the way numpy array orders
            cv2.imwrite('persistent_map.jpg', c)
            '''
        except:
            print("persistent map does not exist, maybe first try")
            b = cv2.imread('ego_map.jpg',0)
            cv2.imwrite('persistent_map.jpg', b)

    def detect_blocks(self,image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('rgb_image.jpg', image)

        get_edge = cv2.Canny(image, 10, 100)
        canny = np.hstack([get_edge])
        cv2.imwrite('canny_image.jpg', canny)

        blurred = cv2.GaussianBlur(image, (3, 3), 0)
        #Detect the ground plane
        cv_image = cv2.threshold(blurred, 30, 40, cv2.THRESH_BINARY)[1]
        cv2.imshow("blocks", cv_image)
        cv2.waitKey(1)
        cv2.imwrite('thresh.jpg', cv_image*255.0)
        #print("Thresholded image ",cv_image)
        return np.array(cv_image).reshape((480,640))

                
if __name__ =='__main__':
    
    s = slam()

    
    #Need to debug this - executing incorrectly- perceived relative positions are all wrong
    trajectory = [[0.0,-0.3,1.57],[0.0,0.0,1.57],[0.3,0.0,1.57],[1.0,0.0,1.57],[1.0,0.0,0.77],[1.0,0.0,0.0],[1.0,-0.3,0.0]]
    #s.send_motion([0.0,-0.3,1.57])
    #s.store_motion()
    #x,y,r = s.localize()
    #t.sleep(1)
    for tr in trajectory:
        #t.sleep(5)
        x,y,r = s.localize()
        s.send_motion([tr[0],tr[1],tr[2]])
        s.ground_project()
        s.register_update(x,y,r)
       
    
    
    '''
    while(True):
        testVar = raw_input("Finished moving? (y/n).")
        if(testVar=='y'):
            print("stopping SLAM")
            break
        s.store_position()
        x,y,r = s.localize()
        s.ground_project()
        s.register_update(x,y,r)
        #s.random_movement()
    '''
    
#Algorithm in brief:
'''
1. Initialize empty persistent map, fix an absolute coordinate where robot starts

2. Obtain egocentric map by fusing depth with image and ground projecting
    Note - Perfect thresholding needed for seperating out ground in image from 
    other collidable objects
    Note - Depth values need clipping to a certain range
    Note - A special tan function is used according to the clipping range of the depth map
    pcd_clipx and pcd_clipz. If those two parameters change then
    project_scale and the tan function woud also have to change
    (tan function is used to compensate for scaling errors of kinect
    -more accurate depth sensing closer and gets worse farther away)

3. Perceive relative change in (x,y,r) from the initial start mapping positions,orientations.
    That is, perceive the current absolute coordinate position,orientation.

4. Using a special patch function, rotate and translate the egocentric map according to
    perceived relative changes, and patch it to the global map.(pixel by pixel- averaging as needed)
    Note- Size of global map increases after this step.
    Note- Cropping needs to be applied to egocentric map after relative rotation
'''

#To do:
'''
1. speed up the nested for loop part in ground_project function, also use stored arrays instead of cv2.imread
2. ability to move by trajectory (commented portion in main- needs debugging)
3. automatic deciding where to move next based on current map
4. remove old unnecessary functions like patch(), shrink(), etc
5. convert this machine to a simulator engine so that a windows computer with the powerful gpu
   can connect to it and receive simulation data. Use :https://pypi.org/project/pysendfile/
   Do the same thing for the small optiplex machine too

Others:

4. Use mask rcnn for image segmentation
5. Use Neural networks for uncertainty removal from depth map/point_cloud (if any)
6. Use the persistent map concept for MADDPG
'''