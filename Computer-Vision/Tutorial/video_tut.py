import numpy as np
import cv2
import time

# cap = cv2.VideoCapture("Video/Megamind.avi")
cap = cv2.VideoCapture(0)

# cap.set(3,320)
# cap.set(4,320)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret==True:
        frame = cv2.flip(frame,0)

        out.write(frame)


    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
        cv2.imshow('frame',frame)
        # cv2.imshow('frame',frame)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break

# When everything done, release the capture
cap.release()
out.release()
cv2.destroyAllWindows()
