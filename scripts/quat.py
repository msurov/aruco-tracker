import numpy as np


def quat_make_continuous(q):
    R'''
        Brief:
        ------
        The function makes a trajectory `q` continuous

        Detailed:
        ------------
        Given array of normalized quaternions `q = [q1, q2, ...]` representing rotations \
        The function changes signs of some elements s.t. its neighbours be close \
        The algorithm uses the fact that normalized quaternions `a` and `-a` represent the same orientation

        Arguments:
        ----------
        `q` is n-elements array of normalized quaternions
    '''
    q = np.array(q)
    assert q.shape[0] > 1
    assert q.shape[1] == 4
    d = np.sum(q[1:,:] * q[0:-1,:], axis=1)
    cnd = d < 0
    cnd = (np.cumsum(cnd) % 2).astype(bool)
    cnd = np.concatenate(([0], cnd))
    sign = 1 - 2 * cnd
    sign = np.reshape(sign, (-1,1))
    return q * sign

def rpy2quat(rpy):
    R'''
        (roll,pitch,yaw) ↦ quat

        Description:
        ------------
        Represent rotation given by `roll,pitch,yaw` angles to normalized quaternion

        Arguments:
        ----------
        `rpy` a vector of three elements `[roll, pitch, yaw]` or n-elements array of such vectors
    '''
    C = np.cos(0.5 * np.array(rpy))
    S = np.sin(0.5 * np.array(rpy))

    cr,cp,cy = C.T
    sr,sp,sy = S.T

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qw, qx, qy, qz]).T

def quat2rpy(q):
    q = quat_normalize(q)
    w, x, y, z = q.T

    # roll
    sr = 2. * (w * x + y * z)
    cr = 1. - 2. * (x**2 + y**2)
    roll = np.arctan2(sr, cr)

    # pitch
    sp = 2. * (w * y - z * x)
    pitch = np.arcsin(np.clip(sp, -1., 1.))

    # yaw
    sy = 2. * (w * z + x * y)
    cy = 1. - 2. * (y**2 + z**2)
    yaw = np.arctan2(sy, cy)

    return np.array([roll, pitch, yaw]).T

def quat_mul(a, b):
    R'''
        a,b ↦ a * b

        Description:
        ------------
        Product of quaternions `a,b`

        Arguments:
        ----------
        `a` quaternion or n-elements array of quaternions \
        `b` quaternion or n-elements array of quaternions
    '''
    aw,ax,ay,az = a.T
    bw,bx,by,bz = b.T
    cw = aw * bw - ax * bx - ay * by - az * bz
    cx = aw * bx + ax * bw + ay * bz - az * by
    cy = aw * by + ay * bw + az * bx - ax * bz
    cz = aw * bz + az * bw + ax * by - ay * bx
    c = np.array([cw,cx,cy,cz]).T
    return c

def quat_conj(q):
    R'''
        q ↦ q'

        Description:
        ------------
        Conjugate quaternion

        Arguments:
        ----------
        `q` quaternion or n-elements array of quaternions
    '''
    j = np.diag([1,-1,-1,-1])
    return q @ j

def quat_conj_mul(a, b):
    R'''
        a,b ↦ a' * b

        Description:
        ------------
        Multiply the conjugation of quaternion `a` by quaternion `b`

        Arguments:
        ----------
        `a` quaternion or n-elements array of quaternions \
        `b` quaternion or n-elements array of quaternions
    '''
    aw,ax,ay,az = a.T
    bw,bx,by,bz = b.T
    cw = aw * bw + ax * bx + ay * by + az * bz
    cx = aw * bx - ax * bw - ay * bz + az * by
    cy = aw * by - ay * bw - az * bx + ax * bz
    cz = aw * bz - az * bw - ax * by + ay * bx
    c = np.array([cw,cx,cy,cz]).T
    return c

def quat_mul_conj(a, b):
    R'''
        a,b ↦ a * b'

        Description:
        ------------
        Multiply quaternion `a` by the conjugation of quaternion `b`

        Arguments:
        ----------
        `a` quaternion or n-elements array of quaternions \
        `b` quaternion or n-elements array of quaternions
    '''
    aw,ax,ay,az = a.T
    bw,bx,by,bz = b.T
    cw =   aw * bw + ax * bx + ay * by + az * bz
    cx = - aw * bx + ax * bw - ay * bz + az * by
    cy = - aw * by + ay * bw - az * bx + ax * bz
    cz = - aw * bz + az * bw - ax * by + ay * bx
    c = np.array([cw,cx,cy,cz]).T
    return c

def quat_mul_vec(q, v):
    R'''
        q,v ↦ q * (0,v)

        Description:
        ------------
        Multiply quaternion `q` by quaternion `(0,v)` composed from zero scalar part and vector `v`

        Arguments:
        ---------
        `q` is a quaternion or n-elements array of quaternions \
        `v` is a 3d vector or n-elements array of vecotrs
    '''
    q = np.array(q)
    v = np.array(v)
    aw,ax,ay,az = q.T
    bx,by,bz = v.T
    cw = -ax * bx - ay * by - az * bz
    cx =  aw * bx + ay * bz - az * by
    cy =  aw * by + az * bx - ax * bz
    cz =  aw * bz + ax * by - ay * bx
    c = np.array([cw,cx,cy,cz]).T
    return c

def quat_vec_mul(v, q):
    R'''
        v,q ↦ (0,v) * q

        Description:
        ------------
        Multiply quaternion `(0,v)` composed from zero scalar part and vector `v` by quaternion `q`

        Arguments:
        ---------
        `v` is a 3d vector or n-elements array of vecotrs \
        `q` is a quaternion or n-elements array of quaternions
    '''
    ax,ay,az = v.T
    bw,bx,by,bz = q.T
    cw = -ax * bx - ay * by - az * bz
    cx =  ax * bw + ay * bz - az * by
    cy =  ay * bw + az * bx - ax * bz
    cz =  az * bw + ax * by - ay * bx
    c = np.array([cw,cx,cy,cz]).T
    return c

def quat_scal_part(q):
    R'''
        q ↦ scal(q)

        Description:
        ------------
        For a given quaternion `q` returns its scalar part

        Arguments:
        ----------
        `q` quaternion or n-elements array of quaternions
    '''
    return q[...,0]

def quat_vec_part(q):
    R'''
        q ↦ vec(q)

        Description:
        ------------
        For a given quaternion `q` returns its vector part

        Arguments:
        ----------
        `q` quaternion or n-elements array of quaternions
    '''
    return np.delete(q, 0, -1)

def quat_normalize(q):
    R'''
        q ↦ q / ||q||

        Description:
        ------------
        for a given nonzero quaternion `q` (or array of quaternions) returns unit quaternion

        Arguments:
        ----------
        `q` quaternion or n-elements array of quaternions
    '''
    q = np.array(q)
    norm_q = np.linalg.norm(q, axis=(-1))
    norm_q = np.reshape(norm_q, q.shape[:-1] + (1,))
    return q / norm_q

def quat_rot_v1(q, v):
    R'''
        q,v ↦ vec(q * (0, v) * q')

        Description:
        ------------
        Rotate a vector `v` by quaternion `q`

        Arguments:
        ----------
        `q` quaternion or n-elements array of quaternions \
        `v` a 3d vector or n-elements array of 3d vectors
    '''
    q = np.array(q)
    v = np.array(v)
    w,x,y,z = q.T
    vx,vy,vz = v.T
    xx = 1 - 2 * y**2 - 2 * z**2
    xy = 2 * x * y - 2 * z * w
    xz = 2 * x * z + 2 * y * w

    yx = 2 * x * y + 2 * z * w
    yy = 1 - 2 * x**2 - 2 * z**2
    yz = 2 * y * z - 2 * x * w

    zx = 2 * x * z - 2 * y * w
    zy = 2 * y * z + 2 * x * w
    zz = 1 - 2 * x**2 - 2 * y**2

    rx = xx * vx + xy * vy + xz * vz
    ry = yx * vx + yy * vy + yz * vz
    rz = zx * vx + zy * vy + zz * vz

    r = np.array([rx, ry, rz]).T
    return r

def quat_rot_v2(q, v):
    R'''
        q,v ↦ vec(q * (0, v) * q')

        Description:
        ------------
        Rotate a vector `v` by quaternion `q`

        Arguments:
        ----------
        `q` quaternion or n-elements array of quaternions \
        `v` a 3d vector or n-elements array of 3d vectors
    '''
    r = quat_mul_conj(quat_mul_vec(q, v), q)
    return quat_vec_part(r)

quat_rot = quat_rot_v2

def quat_from_angleaxis(angle, axis):
    angleshape = np.shape(angle)
    axisshape = np.shape(axis)
    if not angleshape:
        assert axisshape == (3,)
    elif len(axisshape) == 2:
        assert angleshape[0] == axisshape[0]
    axis = np.reshape(axis, (-1, 3))
    angle = np.reshape(angle, (-1, 1))
    norm = np.linalg.norm(axis, axis=1)
    axis = axis / norm[:,np.newaxis]
    angle = np.reshape(angle, (-1, 1))
    w = np.cos(angle/2)
    v = axis * np.sin(angle/2)
    q = np.concatenate((w, v), axis=1)
    if not angleshape:
        return q[0]
    return q


def quat_from_rodrigues(r):
    r = np.array(r)
    angle = np.linalg.norm(r, axis=-1)
    b = angle < 1e-8
    v = np.zeros(r.shape)
    v[:,:] = r[:,:]
    v[b] = [1,0,0]
    return quat_from_angleaxis(angle, v)


def quat_proj(q, l):
    R'''
        Find projection of rotation `q` by axis `l`
    '''
    l = l / np.linalg.norm(l)
    w = q[0]
    v = q[1:4]
    q = np.zeros(4)
    q[0] = w
    q[1:4] = l * (v @ l)
    q = q / np.linalg.norm(q)
    return q


def test_mul():
    a = rpy2quat(np.random.normal(size=(10,3)))
    b = rpy2quat(np.random.normal(size=(10,3)))
    ab = quat_mul(a, b)
    ba = quat_mul(b, a)
    b1 = quat_mul_conj(ba, a)
    conj_b1 = quat_conj_mul(ab, a)
    one = quat_mul(conj_b1, b1)
    assert np.allclose(one, [1, 0, 0, 0])

def test_rotations():
    q = rpy2quat(np.random.normal(size=(3)))
    v = np.random.normal(size=(10,3))
    r = quat_rot_v1(q, v)
    r2 = quat_rot_v2(q, v)
    assert np.allclose(r, r2)
    r3 = np.array([quat_rot(q, tmp) for tmp in v])
    assert np.allclose(r, r3)

    q = rpy2quat(np.random.normal(size=(10,3)))
    v = np.random.normal(size=(10,3))
    r = quat_rot_v1(q, v)
    r2 = quat_rot_v2(q, v)
    assert np.allclose(r, r2)
    r3 = np.array([quat_rot_v1(*tmp) for tmp in zip(q,v)])
    assert np.allclose(r, r3)

    q = rpy2quat(np.random.normal(size=(10,3)))
    v = np.random.normal(size=(3))
    r = quat_rot_v1(q, v)
    r2 = quat_rot_v2(q, v)
    assert np.allclose(r, r2)
    r3 = np.array([quat_rot_v1(tmp, v) for tmp in q])
    assert np.allclose(r, r3)

def test_rotations_compare_perf():
    from time import time
    q = rpy2quat(np.random.normal(size=(100000,3)))
    v = np.random.normal(size=(100000,3))
    t = time()
    r1 = quat_rot_v1(q, v)
    t = time() - t
    print('quat_rot_v1 performed in %fsec' % t)
    t = time()
    r2 = quat_rot_v2(q, v)
    t = time() - t
    print('quat_rot_v2 performed in %fsec' % t)
    assert np.allclose(r1, r2)

if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    test_mul()
    test_rotations()
    test_rotations_compare_perf()
