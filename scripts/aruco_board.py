import cv2
from cv2 import aruco
import matplotlib.pyplot as plt

im = cv2.imread('../data/Image__2017-07-24__22-18-36.png', 0)
dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
corners, ids, rejected = aruco.detectMarkers(im, dictionary)
print corners[0][0]
print corners[0][1]
print corners[0][2]
print corners[0][3]

# board = aruco.GridBoard_create(7, 7, 50., 5., dictionary)
# im = board.draw((4096, 4096))
# print dir(board)
# print help(board.draw)


# dpi = 600
# mmsz = 50
# inchsz = mmsz / 25.4
# pxsz = int(inchsz * dpi)

# im = aruco.drawMarker(dictionary, 38, pxsz)
# cv2.imwrite('38.jpg', im)

# im = aruco.drawMarker(dictionary, 39, pxsz)
# cv2.imwrite('39.jpg', im)

# cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);

# help(plt.imshow)
# plt.imshow(im, cmap='gray', interpolation='nearest')
# plt.show()
# plt.savefig('pattern.png')
