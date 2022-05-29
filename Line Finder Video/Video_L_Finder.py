import cv2
import numpy as np

video = cv2.VideoCapture("lines.mp4")
W = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(W, H)


# roi1 = transform(ROI(gray, np.array([Limits], np.int32)))
# kernel = np.array([[0, -1, 0],
#                    [-1, 5, -1],
#                    [0, -1, 0]])
# roi3 = transform((cv2.filter2D(src=roi1, depth=-5, kernel=kernel)))
# roi2 = cv2.Canny(image=roi1,
#                  threshold1=300,
#                  threshold2=300,
#                  apertureSize=3,
#                  L2gradient=False)

def ROI(img, vertices):
    mask = np.zeros_like(img)
    channel_count = 3
    match_mask_color = (255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def transform(input_t):
    _, input_t, = video.read()

    pts1 = np.float32([[0.57 * W, 0.51 * H], [0.95 * W, 0.92 * H], [0.44 * W, 0.51 * H], [0.05 * W, 0.92 * H]])
    pts2 = np.float32([[600, 0], [600, 400], [0, 0], [0, 400]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    result = cv2.warpPerspective(frame, matrix, (600, 400))
    return result


while True:
    ret, frame = video.read()
    W = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

    Limits = [((0 / 100) * W, (100 / 100) * H), ((35 / 100) * W, (25 / 100) * H), ((65 / 100) * W, (25 / 100) * H),
              ((100 / 100) * W, (100 / 100) * H)]
    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(image=gray,
                      threshold1=300,
                      threshold2=300,
                      apertureSize=5,
                      L2gradient=False)
    roi = ROI(canny, np.array([Limits], np.int32))
    lines = cv2.HoughLines(roi, rho=2, theta=np.pi / 30, threshold=100, lines=np.array([]),
                           srn=None, stn=None, min_theta=0 * np.pi / 180, max_theta=10 * np.pi / 180)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), thickness=3
                     )

    # lines1 = cv2.HoughLinesP(roi,
    #                          rho=10,
    #                          theta=np.pi / 360,
    #                          threshold=100,
    #                          lines=np.array([]),
    #                          minLineLength=100,
    #                          maxLineGap=100)
    # if lines1 is not None:
    #     for line in lines1:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), thickness=2)

    final = cv2.GaussianBlur(frame, (3, 3), 0)
    cv2.imshow("output1", roi)
    cv2.imshow("output0", final)
    # cv2.imshow("output2", canny)
    key = cv2.waitKey(27)
    if key == 27:
        break

video.release()
cv2.destroyAllWindows()
