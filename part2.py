import cv2
import numpy as np
import time 

cap = cv2.VideoCapture('grass_landing.mp4')
red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
white = (255, 255, 255)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("x")
        break

    frame = cv2.resize(frame, (640, 480))
    frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame2 = cv2.inRange(frame2, (20, 150, 0), (40, 255, 255))

    kernel = np.ones((5, 5), np.uint8)
    frame2 = cv2.erode(frame2, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(frame2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    a = list(contours)
    b = []

    for i in a:
        if (len(i) > 35):  # contour를 결정하는 threshold, 클 수록 컨투어개수가 작아짐
            b.append(i)
    contours = b

    area = []  # "값" 을 가져올 리스트 (면적 "값")
    for i in contours:
        integral = cv2.contourArea(i)
        area.append(integral)

    big_area = max(area)
    big_idx = area.index(big_area)

    # 나온 컨투어 중 가장 면적이 넓은 애 하나 지정
    # 그 컨투어의 중앙값을 목표 지점으로 이동
    x, y = 0, 0
    for i in range(len(contours[big_idx])):
        x += contours[big_idx][i][0][0]
        y += contours[big_idx][i][0][1]

    x //= len(contours[big_idx])
    y //= len(contours[big_idx])
    loc = (x, y)

    cv2.drawContours(frame, contours[big_idx], -1, blue, 3)
    cv2.circle(frame, loc, 5, red, 3, cv2.LINE_AA)

    cv2.imshow('video1', frame)
    cv2.imshow('video2', frame2)

    if (cv2.waitKey(10) == ord('q')):
        print("exit")
        break

cap.release()
cv2.destroyAllWindows()