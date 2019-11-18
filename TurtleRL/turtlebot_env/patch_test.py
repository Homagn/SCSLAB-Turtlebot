#from basic_slam import slam
from basic_slam2 import slam
import numpy as np
import cv2
import copy

s = slam()
a = cv2.imread('ego_map.jpg',0)
b = np.zeros_like(a)

#a = cv2.imread('persistent_map.jpg',0)
#b = cv2.imread('ego_map.jpg',0)
#a = np.ones((5,5))
#b = np.ones((5,5))

#Due to a faulty transformation sequence the function is written in
# [y,x,r] needs to be passed in two stages
c = s.patch(b,a,[0,0],-45, testing = True)
d = s.patch(a,c,[0,0],0, testing = True)
print("combined ",d)
cv2.imwrite('patch_test.jpg', d)