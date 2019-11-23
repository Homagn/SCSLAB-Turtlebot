#from basic_slam import slam
from basic_slam2 import slam
import numpy as np
import cv2
import copy
def rotateImage(image,angle):
    image_center = tuple(np.array(image.shape[1::-1])/2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result
s = slam()

o = cv2.imread('ego_map.jpg',0)
o1 = cv2.imread('ego_map1.jpg',0)
o_r = s.patch2(o,o1,[50,0],testing = True)
cv2.imwrite('patch_test.jpg', o_r)


'''
o = cv2.imread('ego_map.jpg',0)
o_r = rotateImage(o,-45)
cv2.imwrite('patch_test.jpg', o_r)
'''

'''
a = cv2.imread('ego_map1.jpg',0)
o = cv2.imread('ego_map.jpg',0)
o2 = cv2.imread('patch_test_shrink.jpg',0)

d = s.patch(o,o2,[0,0],-45, testing = True)
cv2.imwrite('patch_test.jpg', d)
'''

'''
b = np.zeros_like(a)

#a = cv2.imread('persistent_map.jpg',0)
#b = cv2.imread('ego_map.jpg',0)
#a = np.ones((5,5))
#b = np.ones((5,5))

#Due to a faulty transformation sequence the function is written in
# [y,x,r] needs to be passed in two stages
c = s.shrink(a,45)
cv2.imwrite('patch_test_shrink.jpg', c)
#d,_ = s.patch(o,c,[0,0],0,targ_center = [67.2042,2.74], testing = True)
d = s.patch(o,c,[0,0],-45, testing = True)
#d,_ = s.patch(o,c,[0,0],0,targ_center = [57,65], testing = True)
print(a.shape)
print(b.shape)
print(c.shape)
print(d.shape)
print("combined ",d)
cv2.imwrite('patch_test.jpg', d)
'''