import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('IPM test.png')

# def invert(frame):
p1 = 289, 260
p2 = 355, 260
p3 = 108, 440
p4 = 536, 440

cv2.circle(img, p1, 5, (0, 0, 255), -1)
cv2.circle(img, p2, 5, (0, 0, 255), -1)
cv2.circle(img, p3, 5, (0, 0, 255), -1)
cv2.circle(img, p4, 5, (0, 0, 255), -1)
points_first = np.float32([p1, p2, p3, p4])
points_second = np.float32([[0,0], [400,0], [0,600], [400,600]])
matrix = cv2.getPerspectiveTransform(points_first, points_second)
result = cv2.warpPerspective(img, matrix, (400, 600))

plt.imshow(img)
plt.imshow(result)
plt.show()
