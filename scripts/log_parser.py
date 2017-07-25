import numpy as np
import matplotlib.pyplot as plt;
import sys
sys.path.append('../../python-common/src/')
from common import scanf_textfile


data = scanf_textfile('%d [%f, %f, %f]', '../build/log.txt')
data = np.array(data)

r = np.sqrt(data[:,1]**2 + data[:,2]**2 + data[:,3]**2)
plt.plot(r, '.')
plt.grid()
plt.show()

# f, axarr = plt.subplots(nrows=1, ncols=2, sharey=True)
# axarr[0].hist(data[:,2], bins=10)
# axarr[0].grid()
# axarr[1].hist(data[:,3], bins=10)
# axarr[1].grid()
# plt.show()
