import cv2
import numpy as np
import matplotlib.pyplot as plt


def make_rotmat(rodrigues):
    R,_ = cv2.Rodrigues(rodrigues)
    return R


def rotmat_to_rodrigues(R):
    r,_ = cv2.Rodrigues(R)
    r = np.reshape(r, 3)
    return r


def rodrigues_to_angleaxis(r):
    angle = np.linalg.norm(r)
    if abs(angle) < 1e-15:
        return 0., np.array([0.,0.,0.])
    axis = r / angle
    return angle, axis


def wedge(v):
    x,y,z = v
    return np.array([
            [ 0, -z,  y],
            [ z,  0, -x],
            [-y,  x,  0]
        ])


def vee(V):
    return V[[2,0,1],[1,2,0]]


def test_rotmat_approx():
    # small rotation
    r = np.array([2., -0.3, 0.2], dtype=np.float32)
    δr = 0.1 * (np.random.random(3) - 0.5)
    v = np.random.random(3) - 0.5
    v = v / np.linalg.norm(v)
    R1 = make_rotmat(r)
    R2 = R1 @ make_rotmat(δr)
    I = np.eye(3)
    R3 = R1 @ (I + wedge(δr))
    print((R2 - R3) @ v)
    print((R2 - R1) @ v)


def make_transform(r, t):
    R,_ = cv2.Rodrigues(r)
    T = np.eye(4)
    T[0:3,0:3] = R
    T[0:3,3] = t
    return T


def project_points(K, T, pts):
    R = T[0:3,0:3]
    t = T[0:3,3]
    m = [K @ (R @ p + t) for p in pts]
    u = [np.array([x/z, y/z]) for x,y,z in m]
    return np.array(u)


def plot_cont(cont, *wargs, **kwargs):
    pts = np.concatenate((cont, [cont[0]]), axis=0)
    plt.plot(pts[:,0], pts[:,1], *wargs, **kwargs)



def test_pnp_2():
    '''
    for i in range(100):
        δr = 0*np.random.normal(0.0, δr_amplitude, 3)
        δt = np.random.normal(0.0, δt_amplitude, 3)

        δT = make_transform(δr, δt)
        u = project_points(K, T @ δT, pts)
        print(δr)
        plot_cont(u, 'o-', color='b', alpha=0.1)
    
    plt.grid()
    plt.show()
    '''


def test_proj_cov():
    sx,sy = 1600, 1200
    f = 1200
    K = np.array([
        [f, 0, sx/2],
        [0, f, sy/2],
        [0, 0, 1]
    ], dtype=np.float32)
    pts = np.array([
        [   0,    0, 0],
        [8e-2,    0, 0],
        [8e-2, 8e-2, 0],
        [   0, 8e-2, 0]
    ])

    r = np.array([2., -0.3, 0.2], dtype=np.float32)
    t = np.array([0.2, 0.1, 2.], dtype=np.float32)
    T = make_transform(r, t)
    R = T[0:3,0:3]
    t = T[0:3,3]

    # v1
    J = np.zeros((8, 6), np.float)

    for i in range(len(pts)):
        q = pts[i]
        m = K @ (R @ q + t)
        mx,my,mz = m
        A = 1. / mz**2 * np.array([
            [mz, 0, -mx],
            [0, mz, -my]
        ])
        Jr = -A @ K @ R @ wedge(q)
        Jt = A @ K
        J[2*i:2*i+2,0:3] = Jr
        J[2*i:2*i+2,3:6] = Jt

    δr_amplitude = 0.01
    δt_amplitude = 0.001
    cov_rt = np.diag([δr_amplitude,δr_amplitude,δr_amplitude,
        δt_amplitude,δt_amplitude,δt_amplitude])**2

    cov_u = J @ cov_rt @ J.T

    # v2 
    samples = []
    for i in range(100):
        δr = np.random.normal(0.0, δr_amplitude, 3)
        δt = np.random.normal(0.0, δt_amplitude, 3)
        T_var = np.eye(4)
        T_var[0:3,0:3] = R @ make_rotmat(δr)
        T_var[0:3,3] = t + δt
        u = project_points(K, T_var, pts)
        samples += [np.reshape(u, (-1,))]
    
    samples = np.array(samples)
    cov_u_ = np.cov(samples.T)

    print('covariance matrix estimation rel error: ', np.linalg.norm(cov_u - cov_u_) / np.linalg.norm(cov_u))


def test_pnp_jac():
    sx,sy = 1600, 1200
    f = 1200
    K = np.array([
        [f, 0, sx/2],
        [0, f, sy/2],
        [0, 0, 1]
    ], dtype=np.float32)
    distortion = np.zeros(5, dtype=np.float32)
    pts = np.array([
        [   0,    0, 0],
        [8e-2,    0, 0],
        [8e-2, 8e-2, 0],
        [   0, 8e-2, 0]
    ])

    r = np.array([2., -0.3, 0.2], dtype=np.float32)
    t = np.array([0.2, 0.1, 2.], dtype=np.float32)
    T = make_transform(r, t)
    R = T[0:3,0:3]
    t = T[0:3,3]
    u = project_points(K, T, pts)
    u = np.reshape(u, (-1,))

    # v1
    J = np.zeros((8, 6), np.float)

    for i in range(len(pts)):
        q = pts[i]
        m = K @ (R @ q + t)
        mx,my,mz = m
        A = 1. / mz**2 * np.array([
            [mz, 0, -mx],
            [0, mz, -my]
        ])
        Jr = -A @ K @ R @ wedge(q)
        Jt = A @ K
        J[2*i:2*i+2,0:3] = Jr
        J[2*i:2*i+2,3:6] = Jt    

    Jpi = np.linalg.pinv(J)

    # test jacobian
    δr = np.random.normal(0, 0.01, 3)
    δt = np.random.normal(0, 0.001, 3)
    T_var = np.eye(4)
    T_var[0:3,0:3] = R @ make_rotmat(δr)
    T_var[0:3,3] = t + δt
    u1 = project_points(K, T_var, pts)
    u1 = np.reshape(u1, (-1,))
    δu = u1 - u
    δu_ = J @ np.concatenate((δr,δt), axis=0)
    print('jacobian relative tolerance: ', np.linalg.norm(δu - δu_) /  np.linalg.norm(δu))

    # test inv jacobian
    δu = np.random.normal(0, 0.1, 8)
    ok, r1, t1 = cv2.solvePnP(
        np.array(pts, dtype=np.float32),
        np.array(np.reshape(u + δu, (-1,2)), dtype=np.float32),
        K, distortion
    )
    r1 = np.reshape(r1, (-1,))
    t1 = np.reshape(t1, (-1,))
    δr = rotmat_to_rodrigues(R.T @ make_rotmat(r1))
    δt = t1 - t
    δrt = np.concatenate((δr, δt), axis=0)
    δrt_ = Jpi @ δu
    print('inverse jacobian relative tolerance: ', np.linalg.norm(δrt - δrt_) /  np.linalg.norm(δrt))


def test_pnp_cov():
    sx,sy = 1600, 1200
    f = 1200
    K = np.array([
        [f, 0, sx/2],
        [0, f, sy/2],
        [0, 0, 1]
    ], dtype=np.float32)
    distortion = np.zeros(5, dtype=np.float32)
    pts = np.array([
        [   0,    0, 0],
        [8e-2,    0, 0],
        [8e-2, 8e-2, 0],
        [   0, 8e-2, 0]
    ])

    r = np.array([2., -0.3, 0.2], dtype=np.float32)
    t = np.array([0.2, 0.1, 2.], dtype=np.float32)
    T = make_transform(r, t)
    R = T[0:3,0:3]
    t = T[0:3,3]
    u = project_points(K, T, pts)
    u = np.reshape(u, (-1,))

    # v1
    J = np.zeros((8, 6), np.float)

    for i in range(len(pts)):
        q = pts[i]
        m = K @ (R @ q + t)
        mx,my,mz = m
        A = 1. / mz**2 * np.array([
            [mz, 0, -mx],
            [0, mz, -my]
        ])
        Jr = -A @ K @ R @ wedge(q)
        Jt = A @ K
        J[2*i:2*i+2,0:3] = Jr
        J[2*i:2*i+2,3:6] = Jt

    Jpi = np.linalg.pinv(J)

    # v2 
    δu_arr = []
    δrt_arr = []

    for i in range(100):
        δu = np.random.normal(0, 0.2, 8)
        ok, r1, t1 = cv2.solvePnP(
            np.array(pts, dtype=np.float32),
            np.array(np.reshape(u + δu, (-1,2)), dtype=np.float32),
            K, distortion
        )
        if not ok:
            print('can\'t solve')
        r1 = np.reshape(r1, (-1,))
        t1 = np.reshape(t1, (-1,))
        δr = rotmat_to_rodrigues(R.T @ make_rotmat(r1))
        δt = t1 - t
        δrt = np.concatenate((δr, δt), axis=0)

        if np.max(np.abs(δr)) > 0.1:
            print('failed')
            # T1 = make_transform(r1, t1)
            # u1 = project_points(K, T1, pts)
            # u1 = np.reshape(u1, (-1,))
            # print('u =          ', u)
            # print('repr r1,t1 = ', u1)
            # print('u + δu =     ', u + δu)
            # print(Jpi @ δu)
            # print(δrt)
            # print(rodrigues_to_angleaxis(r1))
            # print(rodrigues_to_angleaxis(r))
            # print(rodrigues_to_angleaxis(δr))
            # R1 = make_rotmat(r1)
            ok, r2, t2 = cv2.solvePnP(
                np.array(pts, dtype=np.float32),
                np.array(np.reshape(u + δu, (-1,2)), dtype=np.float32),
                K, distortion, 
                np.array(r, dtype=np.float32),
                np.array(t, dtype=np.float32),
                useExtrinsicGuess=True
            )
            r2 = np.reshape(r2, (-1,))
            t2 = np.reshape(t2, (-1,))
            print(t, t2, t1)
            print(r, r2, r1)
            u1 = project_points(K, make_transform(r1, t1), pts)
            u1 = np.reshape(u1, (-1,))
            err1 = u + δu - u1
            u2 = project_points(K, make_transform(r2, t2), pts)
            u2 = np.reshape(u2, (-1,))
            err2 = u + δu - u2
            print(np.linalg.norm(err1), err1)
            print(np.linalg.norm(err2), err2)

        δu_arr += [δu]
        δrt_arr += [δrt]

    δu_arr = np.array(δu_arr)
    δrt_arr = np.array(δrt_arr)

    cov_u = np.cov(δu_arr.T)
    cov_rt = Jpi @ cov_u @ Jpi.T
    cov_rt_ = np.cov(δrt_arr.T)

    # print(cov_rt)
    # print(cov_rt_)

    print('covariance relative tolerance: ', np.linalg.norm(cov_rt - cov_rt_) / np.linalg.norm(cov_rt_))


def rand_norm(Cov):
    d,_ = np.shape(Cov)
    U,sing,_ = np.linalg.svd(Cov)
    M = np.random.normal(0, 1., d)
    return U @ np.diag(np.sqrt(sing)) @ M


def test_rand_norm():
    J = np.diag([5,1,0.1]) @ (np.random.random((3,3)) - 0.5)
    Cov = J @ J.T
    r = np.array([rand_norm(Cov) for i in range(200)])
    Cov2 = np.cov(r.T)
    print(np.linalg.norm(Cov - Cov2) / np.linalg.norm(Cov))


if __name__ == '__main__':
    np.set_printoptions(suppress=True, linewidth=200)
    # test_proj_cov()
    # test_pnp_jac()
    test_pnp_cov()
    # test_rand_norm()
