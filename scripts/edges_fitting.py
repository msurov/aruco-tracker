import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.linalg import block_diag


class FindMarkers:
    def __init__(self):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

    def find(self, im):
        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
        corners,ids,_ = aruco.detectMarkers(im, self.dictionary, parameters=parameters)
        return list(zip(ids[:,0], corners))


def totuple(arr):
    return tuple(np.reshape(arr, (-1,)))


def fit_line_weighted(pts, w):
    '''
        :param pts: numpy.array of dimension Nx2, points to be fitted
        :param w: numpy.array of dimension N, weights of the points
        :param v: numpy.array of dimension 2, direction vector of the line
        :param p0: numpy.array of dimension 2, displacement of the line
        :param Jac_v: numpy.array of dimension 2xN, Jacobian ∂v/∂w
        :param Jac_p0: numpy.array of dimension 2xN, Jacobian ∂p0/∂w
        :return: v, p0, Jac_v, Jac_p0
    '''
    W = np.sum(w)
    p0 = np.dot(w, pts) / W
    d = pts - p0
    A = np.einsum('i,ij,ik->jk', w, d, d)
    U,sing,Vt = np.linalg.svd(A)
    v = Vt[:,0]
    λ = sing[0]

    Jac_p0 = d.T / W

    S = np.array([[0, -1], [1, 0]], float)
    v_perp = S @ v
    α = -(d @ v_perp) * (d @ v) / (v_perp.T @ A @ v_perp - λ)
    Jac_v = np.outer(v_perp, α.T)

    return v, p0, Jac_v, Jac_p0


def parline_to_impline(v, p0, Jv=None, Jp0=None):
    x0,y0 = p0
    vx,vy = v
    a = vy
    b = -vx
    c = vx*y0 - vy*x0
    l = np.array([a,b,c])

    if Jv is None:
        return l

    J1 = np.array([
        [0, 1],
        [-1, 0],
        [y0, -x0]
    ])
    J2 = np.array([
        [0, 0],
        [0, 0],
        [-vy, vx]
    ])
    J = J1 @ Jv + J2 @ Jp0
    return l, J


def plot_line(p1, p2, **kwargs):
    x1,y1 = p1
    x2,y2 = p2
    plt.plot([x1,x2], [y1,y2], **kwargs)


def make_translation(dx, dy):
    return np.array([
        [1, 0, dx],
        [0, 1, dy],
        [0, 0, 1]
    ])


def make_rotation(alpha):
    c = np.cos(alpha)
    s = np.sin(alpha)
    return np.array([
        [c,-s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])


'''
def refine_edge(im, grad, p1, p2):
    wnd = 20.
    msk = np.zeros(im.shape, dtype='uint8')
    l = p2 - p1
    l = l / np.linalg.norm(l)

    alpha = np.arctan2(l[1], l[0])
    wnd_width = np.linalg.norm(p2 - p1)
    wnd_height = wnd
    wnd_center = (p1 + p2) / 2.
    wnd_sz = int(wnd_width), int(wnd_height)

    T = make_translation(wnd_width / 2., wnd_height / 2.) @ \
        make_rotation(-alpha + np.pi) @ \
        make_translation(-wnd_center[0], -wnd_center[1])

    cropped = cv2.warpAffine(im, T[0:2,:], wnd_sz, cv2.INTER_LINEAR)
    cropped = cv2.Sobel(cropped, -1, 0, 1, scale=0.25)
    y,x = np.nonzero(cropped > 25)
    w = cropped[y,x] / np.float32(255.)
    pts = np.array([x, y]).T
    v, p0, Jv, Jp0 = fit_line_weighted(pts, w)

    S = np.array([[0,-1],[1,0]])
    n = -S @ v
    d = -n.T @ p0
    l = np.array([n[0],n[1],d])
    l = l @ T

    return l
'''


def refine_edge(im, grad, p1, p2):
    wnd = 11
    msk = np.zeros(im.shape, dtype='uint8')
    l = p2 - p1
    l = l / np.linalg.norm(l)
    n = np.array([l[1], -l[0]])

    cv2.line(msk, totuple(p1 + l * wnd / 2), totuple(p2 - l * wnd / 2), 1, wnd)

    grad_n = grad @ n
    grad_n = np.clip(grad_n, 0, np.inf)
    msk = msk & (grad_n > 20.)

    y,x = np.nonzero(msk)
    w = grad_n[msk > 0]

    pts = np.array([x, y]).T
    v, p0, Jac_v_w, Jac_p0_w = fit_line_weighted(pts, w)

    l, Jl = parline_to_impline(v, p0, Jac_v_w, Jac_p0_w)
    Cov_l = Jl @ Jl.T

    return l, Cov_l


def lines_crossing_pt(l1, l2):
    A = np.array([l1[0:2], l2[0:2]])
    b = np.array([-l1[2], -l2[2]])
    x,y = np.linalg.solve(A, b)
    return x,y


def refine_marker(im, grad, marker):
    id,corners = marker
    p1,p2,p3,p4 = np.reshape(corners, (-1,2))

    l1,Cov_l1 = refine_edge(im, grad, p1, p2)
    l2,Cov_l2 = refine_edge(im, grad, p2, p3)
    l3,Cov_l3 = refine_edge(im, grad, p3, p4)
    l4,Cov_l4 = refine_edge(im, grad, p4, p1)

    '''
    print('Cov_l1 =\n', np.linalg.eig(Cov_l1)[0])
    print('Cov_l2 =\n', np.linalg.eig(Cov_l2)[0])
    print('Cov_l3 =\n', np.linalg.eig(Cov_l3)[0])
    print('Cov_l4 =\n', np.linalg.eig(Cov_l4)[0])

    c1,Jac_c1 = impline_crossing_pt(l4, l1)
    Cov_c1 = Jac_c1 @ block_diag(Cov_l4, Cov_l1) @ Jac_c1.T

    c2,Jac_c2 = impline_crossing_pt(l1, l2)
    Cov_c2 = Jac_c2 @ block_diag(Cov_l1, Cov_l2) @ Jac_c2.T

    c3,Jac_c3 = impline_crossing_pt(l2, l3)
    Cov_c3 = Jac_c3 @ block_diag(Cov_l2, Cov_l3) @ Jac_c3.T

    c4,Jac_c4 = impline_crossing_pt(l3, l4)
    Cov_c4 = Jac_c4 @ block_diag(Cov_l3, Cov_l4) @ Jac_c4.T

    print('Cov_c1 =\n', Cov_c1)
    print('Cov_c2 =\n', Cov_c2)
    print('Cov_c3 =\n', Cov_c3)
    print('Cov_c4 =\n', Cov_c4)


    Z = np.zeros((2, 3))
    J = np.block([
        [Jac_11, Z, Z, Jac_14],
        [Jac_21, Jac_22, Z, Z],
        [Z, Jac_32, Jac_33, Z],
        [Z, Z, Jac_43, Jac_44]
    ])
    Cov_l = block_diag(Cov_l1, Cov_l2, Cov_l3, Cov_l4)
    print(Cov_l)
    vals,vecs = np.linalg.eig(Cov_l)
    # print('Cov l =\n', Cov_l)
    # print('Eigval Cov l =', vals)

    Cov_c = J @ Cov_l @ J.T
    c = np.concatenate((c1,c2,c3,c4))
    # print('c =', c)
    # print('Cov_c =\n', Cov_c * 1e+6)
    vals,vecs = np.linalg.eig(Cov_c)
    # print('Eigval Cov c =', np.sqrt(vals))
    '''

    c1,Jac_14,Jac_11 = impline_crossing_pt(l4, l1)
    c2,Jac_21,Jac_22 = impline_crossing_pt(l1, l2)
    c3,Jac_32,Jac_33 = impline_crossing_pt(l2, l3)
    c4,Jac_43,Jac_44 = impline_crossing_pt(l3, l4)

    corners = np.array([c1,c2,c3,c4])

    return id, corners


def refine_markers(im, grad, markers):
    return [refine_marker(im, grad, marker) for marker in markers]


def gen_sample_line(v, p0, npts):
    pts = np.random.random((npts, 2)) - 0.5
    w = np.random.randint(0, 256, (npts,), dtype=np.uint8)
    S = np.array([
        [100.0, 0.0],
        [  0.0, 5.0]
    ])
    R = np.array([
        [v[0],-v[1]],
        [v[1], v[0]]
    ])
    pts = np.array([R @ S @ p + p0 for p in pts])
    return pts, w


def plot_points(pts, weights):
    for p,w in zip(pts, weights):
        plt.plot([p[0]], [p[1]], 'o', alpha=w/255., color='b')


def test_line_fitting():
    np.random.seed(0)
    v = np.random.random(2) - 0.5
    v = v / np.linalg.norm(v)
    p0 = 100 * (np.random.random(2) - 0.5)
    l = parline_to_impline(v, p0)
    print('l =', l)

    NSamples = 100
    samples = []
    amplitude = 5
    pts,w0 = gen_sample_line(v, p0, 200)

    for i in range(NSamples):
        w = cv2.addWeighted(w0, 1,
            np.random.randint(0, amplitude + 1, size=w0.shape, dtype=np.uint8), 1, -amplitude/2.)
        w = np.reshape(w, (-1,))
        w = np.array(w, dtype=np.float32)
        sample = fit_line_weighted(pts, w)
        v,p0,Jv,Jp0 = sample
        l,Jl = parline_to_impline(v, p0, Jv, Jp0)
        samples += [(l, Jl, w)]

    l_arr, Jac_l_arr, w_arr = zip(*samples)
    l_arr = np.array(l_arr)
    l = np.mean(l_arr, axis=0)
    print('l =', l)

    w_arr = np.array(w_arr)
    d = w_arr - np.mean(w_arr, axis=0)
    Cov_w = d.T @ d / len(w_arr)

    Jac_l = np.mean(Jac_l_arr, axis=0)

    Cov_l = np.cov(l_arr.T)
    print('Cov_l =\n', Cov_l)

    Jac_l @ Cov_w @ Jac_l.T
    Cov_l = np.cov(l_arr.T)
    print('Cov_l =\n', Cov_l)


def impline_crossing_pt(line1, line2):
    '''
        :param line1: implicit line, tuple of floats (a,b,c), where a*x+b*y+c = 0 defines a line
        :param line2: implicit line,
        :param p: matrix of dimension 2, coordinates of crossing point
        :param J1: matrix of dimension 2x6, the jacobian ∂p/∂(a1,b1,c1)
        :param J2: matrix of dimension 2x6, the jacobian ∂p/∂(a2,b2,c2)
        :returns: p,J1,J2
    '''
    a1,b1,c1 = line1
    a2,b2,c2 = line2
    A = np.array([
        [a1, b1],
        [a2, b2]
    ])
    B = np.array([
        [-c1],
        [-c2]
    ])
    p = np.linalg.solve(A, B)
    p = p[:,0]
    x,y = p
    D1 = - np.array([
        [x, y, 1],
        [0, 0, 0]
    ])
    D2 = - np.array([
        [0, 0, 0],
        [x, y, 1]
    ])
    J1 = np.linalg.solve(A, D1)
    J2 = np.linalg.solve(A, D2)
    return p, J1, J2


def parline_crossing_pt(line1, line2):
    '''
        :param line1: parametric line, tuple of vectors (v,p0), where p = vt + p0 defines a line
        :param line2: parametric line,
        :param p: matrix of dimension 2, coordinates of crossing point
        :param J: matrix of dimension 2x6, the jacobian ∂p/∂(v1,p1,v2,p2)
        :returns: p,J
    '''
    v1,p1 = line1
    v2,p2 = line2
    S = np.array([
        [0, -1],
        [1, 0]
    ])
    V = np.array([v1, v2]).T
    A = V.T @ S
    B = np.array([
        [v1.T @ S @ p1],
        [v2.T @ S @ p2],
    ])
    p = np.linalg.solve(A, B)
    p = p[:,0]
    x,y = p
    x1,y1 = p1
    x2,y2 = p2
    v1x,v1y = v1
    v2x,v2y = v2
    D = np.array([
        [y-y1, x1-x, v1y, -v1x, 0, 0, 0, 0],
        [0, 0, 0, 0, y-y2, x2-x, v2y, -v2x]
    ])
    J = np.linalg.solve(A, D)
    return p, J


def impline_to_parline(impline):
    '''
        Convert implicit line to parametric
        :param impline: tuple of floats (a,b,c)
        :param parline: tuple of vectors (v,p0):
        :returns: parline
    '''
    a,b,c = impline
    v = np.array([-b, a])
    p0 = -np.array([a, b]) * c
    return v,p0


def conf_ellipse(center, P):
    t = np.linspace(0, 2*np.pi, 300)
    vals,vecs = np.linalg.eig(P)
    v1 = 2.5 * vecs[:,0] * np.sqrt(vals[0])
    v2 = 2.5 * vecs[:,1] * np.sqrt(vals[1])
    pts = np.outer(np.sin(t),v1) + np.outer(np.cos(t),v2) + center
    plt.fill(pts[:,0], pts[:,1], color='orange', alpha=0.5)
    plot_line(center, center + v1, color='orange')
    plot_line(center, center + v2, color='orange')


def test_implines_crossing():
    np.random.seed(0)
    NSamples = 1000
    l1 = np.array([4,8,43])
    l2 = np.array([-6,2,23])
    samples = []

    for i in range(NSamples):
        δl1 = 0.2 * (np.random.random(3) - 0.5)
        δl2 = 0.3 * (np.random.random(3) - 0.5)
        p,J1,J2 = impline_crossing_pt(l1 + δl1, l2 + δl2)
        sample = (l1 + δl1, l2 + δl2, p, J1, J2)
        samples += [sample]

    l1_,l2_,p_,J1_,J2_ = zip(*samples)
    J1_ = np.array(J1_)
    J2_ = np.array(J2_)
    J1 = np.mean(J1_, axis=0)
    J2 = np.mean(J2_, axis=0)
    J = np.concatenate((J1, J2), axis=1)

    l1_ = np.array(l1_)
    l2_ = np.array(l2_)
    tmp = np.concatenate((l1_,l2_), axis=1)
    Cov_l = np.cov(tmp.T)
    Cov_p = J @ Cov_l @ J.T
    print(Cov_p)

    p_ = np.array(p_)
    p = np.mean(p_, axis=0)
    Cov_p = np.cov(p_.T)
    print(Cov_p)

    plt.plot(p_[:,0], p_[:,1], 'o', alpha=0.5)
    plt.axis('equal')
    conf_ellipse(p, Cov_p)
    plt.grid()
    plt.show()


def test_implines_crossing_v2():
    np.random.seed(0)
    l1 = np.array([4.,8.,43.])
    l2 = np.array([-6.,2.,23.])
    δl1 = 0.1 * (np.random.random(3) - 0.5)
    δl2 = 0.2 * (np.random.random(3) - 0.5)

    p,J1,J2 = impline_crossing_pt(l1, l2)
    q,_,_ = impline_crossing_pt(l1 + δl1, l2 + δl2)
    δp_v1 = J1 @ δl1 + J2 @ δl2
    δp_v2 = q - p
    print(δp_v1, δp_v2)


def test_parlines_crossing():
    np.random.seed(0)
    NSamples = 1000
    l1 = np.array([0.5, 0.8]), np.array([32., -43.])
    l2 = np.array([-0.2, 0.6]), np.array([-23., 35.])
    samples = []

    for i in range(NSamples):
        v1 = l1[0] + 0.1 * (np.random.random(2) - 0.5)
        p1 = l1[1] + 1.0 * (np.random.random(2) - 0.5)
        v2 = l2[0] + 0.1 * (np.random.random(2) - 0.5)
        p2 = l2[1] + 1.0 * (np.random.random(2) - 0.5)

        p,J = parline_crossing_pt((v1,p1), (v2,p2))
        sample = (v1,p1,v2,p2,p,J)
        samples += [sample]

    v1_,p1_,v2_,p2_,p_,J_ = zip(*samples)
    tmp = np.concatenate((v1_, p1_, v2_, p2_), axis=1)
    Cov_l = np.cov(tmp.T)
    J_ = np.array(J_)
    J = np.mean(J_, axis=0)

    p_ = np.array(p_)
    p = np.mean(p_, axis=0)
    Cov_p = np.cov(p_.T)
    print(Cov_p)
    Cov_p = J @ Cov_l @ J.T
    print(Cov_p)

    plt.plot(p_[:,0], p_[:,1], 'o', alpha=0.5)
    plt.axis('equal')
    conf_ellipse(p, Cov_p)
    plt.grid()
    plt.show()


def test_parlines_crossing_v2():
    np.random.seed(0)
    v1 = np.array([  0.6,  0.7])
    p1 = np.array([ 16.0,-43.0])
    v2 = np.array([  0.3, -0.9])
    p2 = np.array([-23.0,-35.0])

    δv1 = 0.1 * (np.random.random(2) - 0.5)
    δp1 = 2.1 * (np.random.random(2) - 0.5)
    δv2 = 0.2 * (np.random.random(2) - 0.5)
    δp2 = 1.5 * (np.random.random(2) - 0.5)

    q1,J = parline_crossing_pt((v1, p1), (v2, p2))
    q2,_ = parline_crossing_pt((v1 + δv1, p1 + δp1), (v2 + δv2, p2 + δp2))

    δq = q2 - q1
    δq2 = J @ np.concatenate((δv1, δp1, δv2, δp2))
    print(δq, δq2)


def main():
    im = cv2.imread('tmp/examples-2/20191006_134452.jpg', 0)
    imf = cv2.GaussianBlur(im, (3,3), 1.)
    imf = np.array(imf, dtype=np.float32) / 255
    grad = np.zeros(imf.shape + (2,), float)
    grad[:,:,0] = cv2.Sobel(imf, cv2.CV_32F, 1, 0) / 8.
    grad[:,:,1] = cv2.Sobel(imf, cv2.CV_32F, 0, 1) / 8.

    finder = FindMarkers()
    markers = finder.find(im)
    markers = refine_markers(im, grad, markers)


def draw_quad(quad, imshape):
    p1,p2,p3,p4 = quad
    w = 100
    imsrc = 40*np.ones((w,w), dtype=np.uint8)
    imsrc[1:w,1:w] = 240
    psrc = np.array([
        [    0.5,     0.5],
        [    0.5, w - 0.5],
        [w - 0.5, w - 0.5],
        [w - 0.5,     0.5],
    ], dtype=np.float32)
    pdst = np.array([
        p1, p2, p3, p4
    ], dtype=np.float32)
    H,_ = cv2.findHomography(psrc, pdst)
    im = cv2.warpPerspective(imsrc, H, imshape, borderValue=40)
    return im


def randomize_sample(orig, N, σ):
    samples = []

    for i in range(N - 1):
        im = orig + np.random.normal(0., scale=σ, size=orig.shape)
        im = touint8(im)
        samples += [im]

    return samples


def touint8(arr):
    return np.array(np.clip(arr, 0, 255), np.uint8)


def test_refine_edge():
    np.random.seed(0)
    NSamples = 200

    quad = np.array([[160, 80], [20, 50], [20, 110], [190, 140]], dtype=np.float32)
    p1,p2,p3,p4 = quad
    imshape = (200, 160)
    orig = draw_quad(quad, imshape)
    noise = 5
    samples = randomize_sample(orig, NSamples, noise)
    results = []
    grads = []
    weights = []

    l = parline_to_impline((p2 - p1) / np.linalg.norm(p2 - p1), p1)
    print('l =', l)

    for im in samples:
        blurred = cv2.GaussianBlur(im, (3, 3), 1.)
        blurred = np.array(blurred, dtype=np.float32)
        grad = np.zeros(blurred.shape + (2,), dtype=np.float32)
        grad[:,:,0] = cv2.Sobel(blurred, cv2.CV_32F, 1, 0) / 8.
        grad[:,:,1] = cv2.Sobel(blurred, cv2.CV_32F, 0, 1) / 8.
        l, Cov_l = refine_edge(im, grad, p1, p2)
        results += [(l, Cov_l)]
        grads += [grad]

    l_arr,Cov_l_arr = zip(*results)
    l_arr = np.array(l_arr)
    l = np.mean(l_arr, axis=0)
    print('l =\n', l_arr)
    Cov_l_arr = np.array(Cov_l_arr)
    Cov_l = np.mean(Cov_l_arr, axis=0)
    print('estimated:\n', Cov_l * noise**2)

    Cov_l = np.cov(l_arr.T)
    print('real:\n', Cov_l)


def refine_quad(im, quad):
    p1,p2,p3,p4 = quad

    # blurred = cv2.GaussianBlur(im, (3, 3), 1.)
    blurred = cv2.blur(im, (3,3))
    blurred = np.array(blurred, dtype=np.float32)
    grad = np.zeros(blurred.shape + (2,), dtype=np.float32)
    grad[:,:,0] = cv2.Sobel(blurred, cv2.CV_32F, 1, 0) / 8.
    grad[:,:,1] = cv2.Sobel(blurred, cv2.CV_32F, 0, 1) / 8.

    l1, Cov_l1 = refine_edge(blurred, grad, p1, p2)
    l2, Cov_l2 = refine_edge(blurred, grad, p2, p3)
    l3, Cov_l3 = refine_edge(blurred, grad, p3, p4)
    l4, Cov_l4 = refine_edge(blurred, grad, p4, p1)

    print('line:')
    print(l1)
    print('cov_l1')
    print(Cov_l1)

    print('line:')
    print(l2)
    print('cov_l2')
    print(Cov_l2)

    print('line:')
    print(l3)
    print('cov_l3')
    print(Cov_l3)

    print('line:')
    print(l4)
    print('cov_l4')
    print(Cov_l4)

    c1,Jac_14,Jac_11 = impline_crossing_pt(l4, l1)
    c2,Jac_21,Jac_22 = impline_crossing_pt(l1, l2)
    c3,Jac_32,Jac_33 = impline_crossing_pt(l2, l3)
    c4,Jac_43,Jac_44 = impline_crossing_pt(l3, l4)

    print('corners:')
    print(c1)
    print(c2)
    print(c3)
    print(c4)

    Cov_l = block_diag(Cov_l1, Cov_l2, Cov_l3, Cov_l4)
    Z = np.zeros((2, 3))
    Jac_c_l = np.block([
        [Jac_11, Z, Z, Jac_14],
        [Jac_21, Jac_22, Z, Z],
        [Z, Jac_32, Jac_33, Z],
        [Z, Z, Jac_43, Jac_44]
    ])
    Cov_c = Jac_c_l @ Cov_l @ Jac_c_l.T
    c = np.concatenate((c1,c2,c3,c4))

    return c, Cov_c


def test_refine_quad():
    np.random.seed(0)
    NSamples = 100

    quad = np.array([[160, 80], [20, 50], [16, 110], [190, 140]], dtype=np.float32)
    p1,p2,p3,p4 = quad
    imshape = (200, 160)
    orig = draw_quad(quad, imshape)
    noise = 3
    samples = randomize_sample(orig, NSamples, noise)
    results = []
    grads = []

    for sample in samples:
        c, Cov_c, grad = refine_quad(sample, quad)
        results += [(c, Cov_c)]
        grads += [grad]

    values = np.array([np.reshape(s[90:110,100,0], (-1,)) for s in grads])
    print(values)
    Cov_values = np.cov(values.T)


    c_arr, Cov_c_arr = zip(*results)
    c_arr = np.array(c_arr)

    print('real cov')
    Cov_c = np.cov(c_arr.T)
    print(Cov_c)
    vals,vecs = np.linalg.eig(Cov_c)
    print(np.sqrt(vals))
    print(vecs)

    print('estimated cov')
    Cov_c = np.mean(Cov_c_arr, axis=0) * noise**2
    print(Cov_c)
    vals,vecs = np.linalg.eig(Cov_c)
    print(np.sqrt(vals))
    print(vecs)

    print('estimated c')
    c = np.mean(c_arr, axis=0)
    print(c)

    print('real c')
    c = np.reshape(quad, (-1,))
    print(c)


    plt.imshow(samples[1])
    plt.grid()
    plt.show()


def save_im():
    quad = np.array([[240, 80], [40, 50], [30, 170], [270, 140]], dtype=np.float32)
    p1,p2,p3,p4 = quad
    imshape = (300, 200)
    im = draw_quad(quad, imshape)
    # cv2.imshow('1', im)
    # cv2.waitKey()
    cv2.imwrite('tmp/quad.png', im)


def line_by_points(p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    A = np.array([
        [x1, y1, 1.],
        [x2, y2, 1.]
    ])
    U,sing,Vt = np.linalg.svd(A)
    l = Vt[2,:]
    k = np.linalg.norm(l[0:2])
    l /= k
    return l


def quad_fitting_test():
    im = cv2.imread("tmp/quad.png", 0)
    bin = (im > 128) * np.uint8(255)

    _,contours,_ = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    quad = cv2.approxPolyDP(contours[0], 2, True)
    quad = np.array(quad[:,0,:], dtype=np.float32)
    noise = 3.
    c, Cov_c = refine_quad(im, quad)
    c = np.reshape(c, (-1, 2))
    print('Cov_c')
    print(Cov_c * noise**2)

    print('c')
    print(c)

    '''
    plt.imshow(im, cmap='gray', interpolation='nearest')
    plot_line(c[0], c[1], color='b')
    plot_line(c[1], c[2], color='b')
    plot_line(c[2], c[3], color='b')
    plot_line(c[3], c[0], color='b')

    plot_line([240, 80], [40, 50], color='g')
    plot_line([40, 50], [30, 170], color='g')
    plot_line([30, 170], [270, 140], color='g')
    plot_line([270, 140], [240, 80], color='g')

    plt.show()
    '''


if __name__ == '__main__':
    np.set_printoptions(suppress=False, linewidth=300)

    # save_im()
    quad_fitting_test()
    # test_refine_quad()
    # test_refine_edge()
    # main()
    # test_line_fitting()
    # test_implines_crossing()
    # test_implines_crossing_v2()
    # test_parlines_crossing()
    # test_parlines_crossing_v2()
