import cv2

cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)
    frame_flip = cv2.flip(frame, 1)
    cv2.imshow('Input', frame_flip)

    c = cv2.waitKey(1)
    if c == 101:
        break

cap.release()
cv2.destroyAllWindows()
