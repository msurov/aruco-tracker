import cv2
import matplotlib.pyplot as plt
import numpy as np


def draw_quad(_q):
    q = np.zeros((len(_q) + 1, 2))
    q[:-1,:] = _q
    q[-1,:] = _q[0]
    plt.plot(q[:,0], q[:,1])


def homography_pts(_H, _pts):
    pts = np.array([_pts], dtype='float32')
    H = np.array(_H, dtype='float32')
    res = cv2.perspectiveTransform(pts, H)
    res = res[0,:,:]
    return res


def draw_line(p1,p2,**kwargs):
    plt.plot([p1[0],p2[0]], [p1[1],p2[1]], **kwargs)


def draw_quad_grid(_q):
    q = np.array(_q, dtype='float32')
    s = np.array([
        [0, 0],
        [1, 0],
        [1, 1],
        [0, 1]
    ], dtype='float32')
    H,_ = cv2.findHomography(s, q)
    for i in range(0,8):
        p1 = [0, i/7.]
        p2 = [1, i/7.]
        p1,p2 = homography_pts(H, [p1,p2])
        draw_line(p1, p2, color='b')

        p1 = [i/7., 0.]
        p2 = [i/7., 1.]
        p1,p2 = homography_pts(H, [p1,p2])
        draw_line(p1, p2, color='b')


def L2grad(im):
    dx = cv2.Sobel(im, cv2.CV_32F, 1, 0)
    dy = cv2.Sobel(im, cv2.CV_32F, 0, 1)
    out = np.clip(np.sqrt(dx**2 + dy**2), 0, 255)
    out = np.array(out, dtype='uint8')
    return out


def cont_perim(cont):
    c = np.reshape(cont, (-1,2))
    d = np.zeros(c.shape, dtype=float)
    d[:-1,:] = np.diff(c, axis=0)
    d[-1,:] = c[0] - c[-1]
    d = np.linalg.norm(d, axis=1)
    return np.sum(d)


def cont_area(cont):
    c = np.reshape(cont, (-1,2))
    x = c[:,0]
    y = c[:,1]
    dx = np.zeros(len(x), dtype=float)
    dx[:-1] = np.diff(x)
    dx[-1] = x[0] - x[-1]
    return np.sum(y * dx)


def simplify_cont(cont):
    p = cont_perim(cont)
    side = p / 4.
    eps = side * 3. / 100.
    return cv2.approxPolyDP(cont, eps, True)



def search_markers():
    im = cv2.imread(R'../dataset/00061.png', 0)
    im = cv2.equalizeHist(im)
    # im = cv2.medianBlur(im, 3)

    # blurred = cv2.blur(im, (50,50))
    # im = cv2.addWeighted(im, 1.5, blurred, -1.5, 128)
    # g = L2grad(im)
    binary = np.array(im > 128, dtype='uint8')
    # binary = cv2.adaptiveThreshold(im, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 51, 5)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    contours = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[1]

    def f1(c):
        a = abs(cont_area(c))
        p = cont_perim(c)
        if a < 400: return False
        if p < 100: return False
        if a / p**2 < 1./50: return False
        return True

    contours = filter(f1, contours)
    contours = [simplify_cont(c) for c in contours]
    contours = filter(lambda c: len(c) == 4, contours)

    plot = np.zeros((im.shape[0], im.shape[1], 3), dtype='uint8')
    plot[:,:,0] = im
    plot[:,:,1] = im
    plot[:,:,2] = im
    cv2.drawContours(plot, contours, -1, (0,255,0))

    cv2.namedWindow('1', 0)
    cv2.imshow('1', plot)
    cv2.waitKey()


def draw_markers():
    quads = [
        # [[402.636, 83.4994], [404.486, 163.762], [332.967, 166.978], [329.702, 86.8193]],
        # [[127.742, 867.183], [213.899, 860.127], [212.014, 902.107], [118.082, 910.056]],

        # [[402.303, 83.7715], [404.399, 163.717], [332.951, 166.943], [329.953, 87.0104]],
        # [[127.754, 867.262], [213.542, 860.358], [211.861, 901.853], [118.182, 909.908]],

        # [[402.234, 83.8789], [404.292, 163.652], [332.958, 166.943], [330.052, 87.0791]],
        # [[127.81, 867.343], [213.442, 860.448], [211.748, 901.748], [118.248, 909.828]],

        [[662.276, 351.722], [745.75, 435.553], [673.96, 506.762], [587.111, 423.338]],
        # [[799.998, 481.297], [846.788, 474.65], [774.033, 547.044], [728.764, 551.811]],
        # [[1906.22, 1268.4], [1849.83, 1200.1], [1947.11, 1156.51], [2004.96, 1222.69]],
        # [[1019.51, 615.101], [1000.86, 581.993], [1079.53, 559.811], [1099.56, 590.984]],
        # [[1764.47, 1334.84], [1711.59, 1261.67], [1813.36, 1216.26], [1868.61, 1285.98]],
        # [[1696.13, 626.299], [1661.01, 591.11], [1733.72, 568.331], [1770, 602.495]],

        [[587.57373, 423.741882], [662.380676, 351.819977], [745.314453, 435.049133], [672.440613, 505.320312]]
    ]
    im = cv2.imread(R'../dataset/00061_ac.png')
    plt.imshow(im, cmap='gray', interpolation='nearest')
    for q in quads:
        draw_quad_grid(q)
    plt.show()


search_markers()

