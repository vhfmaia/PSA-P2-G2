import cv2
import numpy as np

video = cv2.VideoCapture("Input.mp4")


def ROI(img, vertices):
    mask = np.zeros_like(img)
    channel_count = 3
    match_mask_color = (255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def invert(img):
    p1 = 289, 260
    p2 = 355, 260
    p3 = 108, 440
    p4 = 536, 440

    # cv2.circle(img, p1, 5, (0, 0, 255), -1)
    # cv2.circle(img, p2, 5, (0, 0, 255), -1)
    # cv2.circle(img, p3, 5, (0, 0, 255), -1)
    # cv2.circle(img, p4, 5, (0, 0, 255), -1)
    points_first = np.float32([p1, p2, p3, p4])
    points_second = np.float32([[0, 0], [400, 0], [0, 600], [400, 600]])
    matrix = cv2.getPerspectiveTransform(points_first, points_second)
    result = cv2.warpPerspective(img, matrix, (400, 600))
    return result

while True:
    ret, frame = video.read()
    if not ret:
        video = cv2.VideoCapture("road_car_view.mp4")
        continue

    W = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    Point_1 = ((5 / 100) * W, (92 / 100) * H)
    Point_2 = ((44 / 100) * W, (519 / 1000) * H)
    Point_3 = ((56 / 100) * W, (519 / 1000) * H)
    Point_4 = ((95 / 100) * W, (92 / 100) * H)
    Limits = [(Point_1, Point_2, Point_3, Point_4 )]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    roi1 = invert(ROI(gray, np.array([Limits], np.int32)))
    kernel = np.array([[0, -1, 0],
                       [-1, 5, -1],
                       [0, -1, 0]])
    roi3 = cv2.filter2D(src=roi1, ddepth=-5, kernel=kernel)
    roi2 = cv2.Canny(image=roi1,
                     threshold1=300,
                     threshold2=300,
                     apertureSize=3,
                     L2gradient=False)

    canny = cv2.Canny(image=gray,
                      threshold1=300,
                      threshold2=300,
                      apertureSize=5,
                      L2gradient=False)
    blur = cv2.GaussianBlur(canny, (7, 7), 0)
    roi = ROI(blur, np.array([Limits], np.int32))
    lines = cv2.HoughLinesP(roi,
                            rho=1,
                            theta=np.pi / 180,
                            threshold=100,
                            lines=np.array([]),
                            minLineLength=1,
                            maxLineGap=1)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), thickness=1)

    lines1 = cv2.HoughLinesP(roi,
                             rho=1,
                             theta=np.pi / 180,
                             threshold=100,
                             lines=np.array([]),
                             minLineLength=1,
                             maxLineGap=1)
    if lines1 is not None:
        for line in lines1:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), thickness=1)

    final = cv2.GaussianBlur(frame, (3, 3), 0)
    cv2.imshow("output1", roi3)
    cv2.imshow("output0", final)
    cv2.imshow("output2", canny)
    key = cv2.waitKey(27)
    if key == 27:
        break

video.release()
cv2.destroyAllWindows()
