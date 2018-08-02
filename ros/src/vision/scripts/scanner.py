import cv2
import os
import sys
import numpy as np

cap = cv2.VideoCapture('cft.mp4')

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        continue
    # resize and gray
    w, h = 550, 400
    sized_img = cv2.resize(img, (w, h))
    img_pp = sized_img
    img_pp = cv2.GaussianBlur(img_pp, (9,9), 0)
    img_pp = cv2.medianBlur(img_pp, 5)

    # canny edge detection
    img_pp = cv2.Canny(img_pp, threshold1=75, threshold2=110)

    dst = cv2.reduce(img_pp, 0, cv2.REDUCE_SUM, dtype=cv2.CV_32S)[0]

    threshold = 3000
    centers = []
    start = -1

    for i, val in enumerate(dst):
        if val >= threshold:
            if start == -1:
                start = i
            else:
                if start != -1:
                    centers.append((sum(dst[start:i]), start, i))
                    start = -1

    if len(centers) < 2:
        continue

    centers = sorted(centers, key=lambda x: x[0], reverse=True)
    diff = abs((centers[0][0] / centers[1][0]) - 1)
    if diff > 1:
        continue
    else:
        x = (centers[0][1] + centers[0][2] + centers[1][1] + centers[1][2]) / 4
        cv2.line(sized_img, (x, 0), (x, h), 3)
    #bgr_img = cv2.cvtColor(img_pp, cv2.COLOR_GRAY2BGR)
    cv2.imshow('detect', sized_img)
    #if cv2.waitKey(10) & 0xFF == ord('q'):
     #   break
    cv2.waitKey(1)

cv2.destroyAllWindows()

