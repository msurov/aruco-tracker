import numpy as np
from scipy.linalg import expm, logm

def wedge(v):
    x,y,z = v
    return np.array([
            [ 0, -z,  y],
            [ z,  0, -x],
            [-y,  x,  0]
        ])


def vee(V):
    return V[[2,0,1],[1,2,0]]


def rodrigues_to_rotmat(r):
    return expm(wedge(r))


def rotmat_to_rodrigues(R):
    return vee(logm(R))


def make_transform(r, t):
    R,_ = cv2.Rodrigues(r)
    T = np.eye(4)
    T[0:3,0:3] = R
    T[0:3,3] = t
    return T


def inv_transform(R, t):
    inv_R = R.T
    inv_t = -R.T @ t
    return inv_R, inv_t


def main():
    r1 = np.random.normal(0, 1., 3)
    R1 = rodrigues_to_rotmat(r1)
    t1 = np.random.normal(0, 0.1, 3)

    δr = np.random.normal(0, 0.01, 3)
    δR = rodrigues_to_rotmat(δr)
    δt = np.random.normal(0, 0.001, 3)
    δrt = np.concatenate((δr, δt), axis=0)

    R2 = R1 @ δR
    r2 = rotmat_to_rodrigues(R2)
    t2 = t1 + δt

    inv_R1, inv_t1 = inv_transform(R1, t1)
    inv_R2, inv_t2 = inv_transform(R2, t2)

    δt_inv = inv_t2 - inv_t1
    δr_inv = rotmat_to_rodrigues(inv_R1.T @ inv_R2)

    print(δt_inv)
    print(wedge(inv_t1) @ δr - R1.T @ δt)


if __name__ == '__main__':
    main()
