import numpy as np
import cv2
import re
import os


samples_dir = '../data/'
files = os.listdir(samples_dir)

for f in files:
	ans = re.match(r'Explorer_HD2K_(.*)\.png', f)
	if ans is None:
		continue

	f = os.path.join(samples_dir, f)
	im = cv2.imread(f)
	h,w,_ = im.shape

	leftname = os.path.join(samples_dir, ans.group(1) + '-left.png')
	rightname = os.path.join(samples_dir, ans.group(1) + '-right.png')

	imleft = im[:,:w/2,:]
	imright = im[:,w/2:,:]

	cv2.imwrite(leftname, imleft)
	cv2.imwrite(rightname, imright)

