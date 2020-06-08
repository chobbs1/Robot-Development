import cv2
import numpy as np

def nothing(x):
    pass

name = 'Trackpad Demo'
ESC = 27

    # Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
cv2.namedWindow(name)

# create trackbars for color change
cv2.createTrackbar('R',name,0,255,nothing)
cv2.createTrackbar('G',name,0,255,nothing)
cv2.createTrackbar('B',name,0,255,nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, name,0,1,nothing)

while(1):
    cv2.imshow(name,img)
    k = cv2.waitKey(1) & 0xFF
    if k == ESC:
        break

    # get current positions of four trackbars
    r = cv2.getTrackbarPos('R',name)
    g = cv2.getTrackbarPos('G',name)
    b = cv2.getTrackbarPos('B',name)
    s = cv2.getTrackbarPos(switch,name)

    if s == 0:
        img[:] = 0
    else:
        img[:] = [b,g,r]

cv2.destroyAllWindows()
