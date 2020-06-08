import numpy as np
import cv2
from matplotlib import pyplot as plt

img = np.zeros((512,512,3), np.uint8)
img = cv2.line(img,(0,0),(511,511),(255,255,255),10)
img = cv2.circle(img,(250,100), 50, (0,0,255), -1)

img = cv2.ellipse(img,(256,256),(25,50),0,0,360,255,-1)

pts = np.array([[100,5],[200,5],[200,20],[100,20]], np.int32)
pts = pts.reshape((-1,1,2))
img = cv2.polylines(img,[pts],True,(0,255,255))


font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(img,'Ya mum',(10,500), font, 2,(255,255,255),2,cv2.LINE_AA)


plt.imshow(img)
plt.show()
