from basic_slam import slam
import numpy as np
import cv2
import copy

s = slam()
a = cv2.imread('ego_map.jpg',0)
b = copy.copy(a)

a = cv2.imread('persistent_map.jpg',0)
b = cv2.imread('ego_map.jpg',0)
#a = np.ones((5,5))
#b = np.ones((5,5))

c = s.patch(copy.copy(b),b,[0,0],90)
print("combined ",c)
cv2.imwrite('patch_test.jpg', c)