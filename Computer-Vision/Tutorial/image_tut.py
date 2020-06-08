import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('falcon_heavy.jpg')
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

e1 = cv2.getTickCount()
# your code execution


rocket = img[22:123,80:100]
img[22:123,40:60] = rocket


e2 = cv2.getTickCount()
time = (e2 - e1)/ cv2.getTickFrequency()
print('Time take: {}ns'.format(time*10**6))

# for i in range(50):
#     img.itemset((i,10,1),122)

# print(img.item(px))



plt.imshow(img)
plt.show()
