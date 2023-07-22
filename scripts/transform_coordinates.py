import numpy as np
from scipy.linalg import expm, logm

def wedge(a):
	x,y,z = a
	return np.array([
		[ 0, -z,  y],
		[ z,  0, -x],
		[-y,  x,  0]
	])

def vee(A):
	return A[[2,0,1], [1,2,0]]

np.set_printoptions(suppress=True)
r = np.array([0.7, -1.5, -0.9])
R = expm(wedge(r))
r2 = vee(logm(R))

S = (R - R.T) / 2
v = np.array([S[2,1], S[0, 2], S[1, 0]])
# nv = np.linalg.norm(v)
# print(nv)
# theta = np.arcsin(nv)
# r = v / nv
# print(r * theta)
# print(v * np.sin(theta) / nv)

l = v / np.linalg.norm(v)
theta = np.arccos((np.trace(R) - 1) / 2)

print(l * theta)