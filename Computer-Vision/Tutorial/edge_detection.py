import cv2
import numpy as np
from matplotlib import pyplot as plt

filename = 'apollo11.jpg'
img = cv2.imread(filename)

scale_factor = 2
height, width = img.shape[:2]
scaled_dims = (int(scale_factor*width),int(scale_factor*height))
img = cv2.resize(img,scaled_dims, interpolation = cv2.INTER_AREA)

img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


img_gray = np.float32(img_gray)
dst = cv2.cornerHarris(img_gray,2,3,0.04)


dst = cv2.dilate(dst,None)

img[dst>0.01*dst.max()]=[0,0,255]

cv2.imshow('dst',img)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()
