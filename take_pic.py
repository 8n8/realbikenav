""" To test taking a webcam picture. """

import cv2  # type: ignore

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

cv2.imshow('img1', frame)
cv2.waitKey(0)

cap.release()
