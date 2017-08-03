import cv2
from sys import argv
from os.path import split
import numpy as np


def compare(im1, im2):
	if im1.shape != im2.shape:
		return -1

	d = cv2.absdiff(im1, im2)
	d = np.max(d, axis=2)
	d = cv2.medianBlur(d, 15)
	w,h = d.shape
	z = np.zeros((w,h,3), dtype='uint8')
	print dir(cv2)
	d = z / 2 + im1 / 2
	cv2.imshow('1', d)
	cv2.waitKey()


im1 = cv2.imread('../data/SN13535_20-06-14-left.png')
im2 = cv2.imread('../data/SN13535_20-06-43-left.png')
compare(im1, im2)

if __name__ == '__main__':
	if len(argv) == 1:
		print 'usage: %s im1 im2' % split(argv[0])[1]
		exit(0)

	if len(argv) != 3:
		print 'incorrect usage'
		exit(-1)

	path1 = argv[1]
	path2 = argv[2]

	im1 = cv2.imread(path1)
	if im1 is None:
		print 'can\'t read ' + path1

	im2 = cv2.imread(path2)
	if im2 is None:
		print 'can\'t read ' + path2

	compare(im1, im2)
