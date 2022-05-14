import cv2
import numpy as np

video = cv2.VideoCapture("Video.mp4")

def ROI(img, vertices):
    mask = np.zeros_like(img)
    channel_count = video.shape[2]
    match_mask_color = (255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


while True:
    ret, frame = video.read()
    if not ret:
        video = cv2.VideoCapture("road_car_view.mp4")
        continue

    H = video.shape[0]
    W = video.shape[1]
    Limits = [((0) * W, (1) * H), ((1 / 2) * W, (1 / 2) * H), ((1) * W, (1) * H)]
    roi = ROI(frame, np.array([Limits], np.int32))
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    hsv = cv2.cv2Color(blur, cv2.COLOR_BGR2HSV)
    low_yellow = np.array([18, 94, 140])
    up_yellow = np.array([48, 255, 255])
    mask = cv2.inrange(hsv, low_yellow, up_yellow)
    edges = cv2.Canny(mask, 75, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, maxLineGap=50)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 0)
    cv2.imshow("frame", frame)
    cv2.imshow("edges", edges)
    key = cv2.waitKey(27)
    if key == 27:
        break

video.release()
cv2.destroyAllWindows()