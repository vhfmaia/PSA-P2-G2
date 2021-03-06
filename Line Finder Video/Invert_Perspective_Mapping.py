#!/usr/bin/env python3


# Libraries
import cv2
import numpy as np
import matplotlib.pyplot as plt


# Image path
img_str = "IPM test.png"


# Function: save 4 points and exit
def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        global points

        # Save click coordinates
        if len(points) < 4:
            points.append([x, y])
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

            if len(points) == 4:
                print("4 points saved - click to continue\n")

        # When complete, break loop
        else:
            global running
            running = False


# Get 4 points
points = []
img = cv2.imread(img_str)
running = True
while running:
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', click_event)
    cv2.waitKey(1)


# Second window: get window size
img2 = cv2.imread(img_str)
h = img2.shape[0]
w = img2.shape[1]


# Save to txt file
# point_list = [[w, h], points]
# text_file = open("transform_points.txt", "w")
# text_file.write(str(point_list))
# text_file.close()


# Print relative coordinates
print("(w, h) = " + str(w) + ", " + str(h))
for i in range(len(points)):
    x_relative = points[i][0] / w
    y_relative = points[i][1] / h

    print("Point " + str(i+1) + ": "
          + str(points[i][0]) + ", " + str(points[i][1]) + "  -->  "
          + str(round(x_relative, 3)) + ", " + str(round(y_relative, 3)))


# Second window: preview transform
points_first = np.float32(points)
points_second = np.float32([[0, 0], [h, 0], [0, w], [h, w]])
matrix = cv2.getPerspectiveTransform(points_first, points_second)
result = cv2.warpPerspective(img2, matrix, (w, h))
plt.imshow(img2)
plt.imshow(result)
plt.show()
