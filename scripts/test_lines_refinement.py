import matplotlib.pyplot as plt
import cv2
import numpy as np



def plot_poly(p, **kwargs):
    _p = np.concatenate((p,[p[0]]))
    plt.plot(_p[:,0], _p[:,1], **kwargs)


def main():
    im = cv2.imread('./tmp/examples-7/1.bmp', 0)
    plt.imshow(im, cmap='gray', interpolation='nearest')
    plot_poly([
        [903.4320792947736, 718.9250155033044],
        [969.3731818312697, 781.5803130632695],
        [899.1993858397915, 848.9968228989446],
        [832.8510832542771, 781.0461363416827]])
    plt.show()


if __name__ == '__main__':
    main()
