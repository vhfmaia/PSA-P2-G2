import cv2
import numpy as np

video = cv2.VideoCapture("2022-05-14 12-22-28.mp4")

def ROI(img, vertices):
    mask = np.zeros_like(img)
    channel_count = 3
    match_mask_color = (255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


while True:
    ret, frame = video.read()
    if not ret:
        video = cv2.VideoCapture("road_car_view.mp4")
        continue

    W = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    Limits = [((0) * W, (1) * H), ((1 / 2) * W, (11/20) * H), ((1) * W, (1) * H)]
    blur = cv2.GaussianBlur(frame, (7,7), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    roi1 = ROI(gray, np.array([Limits], np.int32))
    canny = cv2.Canny(image=gray,
                      threshold1=300,
                      threshold2=300,
                      apertureSize=5,
                      L2gradient=False)
    blur = cv2.GaussianBlur(canny, (7, 7), 0)
    roi = ROI(blur, np.array([Limits], np.int32))
    lines = cv2.HoughLinesP(roi,
                            rho=1,
                            theta=np.pi/180,
                            threshold=100,
                            lines=np.array([]),
                            minLineLength=1,
                            maxLineGap=1)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), thickness=1)
    final = cv2.GaussianBlur(frame, (3,3), 0)
    cv2.imshow("output", roi1)
    cv2.imshow("input", final)
    key = cv2.waitKey(27)
    if key==27:
        break

video.release()
cv2.destroyAllWindows()