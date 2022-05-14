import matplotlib.pyplot as plt
import cv2
import numpy as np


def ROI(img, vertices):
    mask = np.zeros_like(img)
    channel_count = image.shape[2]
    match_mask_color = (255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw(img, lines):
    img = np.copy(img)
    line_image = np.zeros((img.shape[0], img.shape[1], img.shape[2]), dtype=np.uint8)

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=3)
    img = cv2.addWeighted(img, 0.8, line_image, 1, 0.0)
    return img

image = cv2.imread('00006.png')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

print(image.shape)
H = image.shape[0]
W = image.shape[1]

ROI_Limits = [((0)*W, (1)*H), ((1/2)*W, (1/2)*H), ((1)*W, (1)*H)]

image0 = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
image1 = cv2.Canny(image0,100, 200)
image2 = ROI(image1, np.array([ROI_Limits], np.int32))

lines = cv2.HoughLinesP(image2,
                        rho=1,
                        theta=np.pi/180,
                        threshold=100,
                        lines=np.array([]),
                        minLineLength=50,
                        maxLineGap=100)
image3 = draw(image, lines)
plt.imshow(image3)
plt.show()