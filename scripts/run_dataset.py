import numpy as np
import subprocess
import glob

cfg_path = '../configs/tracker.json'
dataset_path = '/home/msurov/dev/datasets/calib/SN13535_*-left.png'
app_path = '../build/aruco_tracker-Desktop-Debug/aruco_tracker'


def run_sample(im):
	subprocess.call([app_path, '-c', cfg_path, '-i', im])


def run_all(imgs):
	for im in imgs:
		print 'processing', im
		run_sample(im)

imgs = glob.glob(dataset_path)
run_all(imgs)