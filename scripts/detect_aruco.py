import cv2
from cv2 import aruco
import numpy as np


im = cv2.imread('../data/SN13535_20-06-14-left.png', 0)
dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

im_blurred = cv2.GaussianBlur(im, (15,15), 1.)
corners,ids,rejected_corners = aruco.detectMarkers(im_blurred, dictionary)
corners = np.array(corners, dtype='float32')
term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01)
for i in range(len(corners)):
	corners[i] = cv2.cornerSubPix(im, corners[i], (10,10), (-1,-1), term)

plot = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
plot[:,:,2] = aruco.drawDetectedMarkers(im, corners)
cv2.imshow('1', plot)
cv2.waitKey()

# board = aruco.GridBoard_create(9, 9, 0.08, 0.005, dictionary);
# corners2, ids2, rejected_corners2, recovered_ids2 = aruco.refineDetectedMarkers(im, board, corners, ids, rejected_corners)
# print corners2

# print corners
