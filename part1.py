import cv2
import numpy as np
from matplotlib import pyplot as plt

cap = cv2.VideoCapture('tree_road_grass.mp4')
red = (0, 0, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
white = (255, 255, 255)
frame_list = []

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("x")
        break

    frame = cv2.resize(frame, (640, 480))
    frame2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame2 = cv2.inRange(frame2, (20, 150, 0), (40, 255, 255))

    # 모폴로지컬로 연속적으로 잡히지 않는 나무 영역을 제거하였음 실제로는 detection이 안될거 같긴함
    kernel = np.ones((3, 3), np.uint8)
    frame2 = cv2.erode(frame2, kernel, iterations=2)

    w = [0, int(frame2.shape[1] * 0.25), int(frame2.shape[1] * 0.5), int(frame2.shape[1] * 0.75), frame2.shape[1]]  # x
    h = [0, int(frame2.shape[0] * 0.25), int(frame2.shape[0] * 0.5), int(frame2.shape[0] * 0.75), frame2.shape[0]]  # y

    # 16개로 잘라서 잔디로 찾아진 영역 (255로 차 있는 곳)

    region11 = frame2[h[0]: h[1], w[0]: w[1]]
    region12 = frame2[h[0]: h[1], w[1]: w[2]]
    region13 = frame2[h[0]: h[1], w[2]: w[3]]
    region14 = frame2[h[0]: h[1], w[3]: w[4]]

    region21 = frame2[h[1]: h[2], w[0]: w[1]]
    region22 = frame2[h[1]: h[2], w[1]: w[2]]
    region23 = frame2[h[1]: h[2], w[2]: w[3]]
    region24 = frame2[h[1]: h[2], w[3]: w[4]]

    region31 = frame2[h[2]: h[3], w[0]: w[1]]
    region32 = frame2[h[2]: h[3], w[1]: w[2]]
    region33 = frame2[h[2]: h[3], w[2]: w[3]]
    region34 = frame2[h[2]: h[3], w[3]: w[4]]

    region41 = frame2[h[3]: h[4], w[0]: w[1]]
    region42 = frame2[h[3]: h[4], w[1]: w[2]]
    region43 = frame2[h[3]: h[4], w[2]: w[3]]
    region44 = frame2[h[3]: h[4], w[3]: w[4]]

    region_list = [region11, region12, region13, region14, region21, region22, region23, region23, \
                   region31, region32, region33, region34, region41, region42, region43, region44]

    mean_list = [np.mean(region11), np.mean(region12), np.mean(region13), np.mean(region14), \
                 np.mean(region21), np.mean(region22), np.mean(region23), np.mean(region24), \
                 np.mean(region31), np.mean(region32), np.mean(region33), np.mean(region34), \
                 np.mean(region41), np.mean(region42), np.mean(region43), np.mean(region44)]

    maxm = max(mean_list)
    max_idx = mean_list.index(maxm)

    h_idx = max_idx // 4
    w_idx = max_idx % 4

    y1, y2 = h[h_idx], h[h_idx + 1]
    x1, x2 = w[w_idx], w[w_idx + 1]

    loc = ((x2 - x1) // 2 + x1, (y2 - y1) // 2 + y1)


    cv2.rectangle(frame2, (x1, y1), (x2, y2), red, 3, cv2.LINE_8)
    cv2.rectangle(frame, (x1, y1), (x2, y2), red, 3, cv2.LINE_8)

    
    cv2.circle(frame2, loc, 10, red, 3, cv2.LINE_AA)
    cv2.circle(frame, loc, 10, red, 3, cv2.LINE_AA)

    cv2.imshow('video1', frame)
    cv2.imshow('video2', frame2)

    if (cv2.waitKey(10) == ord('q')):
        print("exit")
        break

cap.release()
cv2.destroyAllWindows()