import socket
import json
import numpy as np
import matplotlib.pyplot as plt
import cv2


def rodrigues_to_rotmat(r):
    R,_ = cv2.Rodrigues(r)
    return R

def maketransform(p, r):
    T = np.eye(4)
    T[0:3,0:3] = rodrigues_to_rotmat(r)
    T[0:3,3] = p
    return T

def decomposetranmsform(T):
    r,_ = cv2.Rodrigues(T[0:3,0:3])
    r = np.reshape(r, (3,))
    p = T[0:3,3]
    return p,r


class Signal:
    def __init__(self, t0, u0, wndsz=20):
        self.t = np.array([t0] * wndsz)
        self.u = np.array([u0] * wndsz)

    def append_measurement(self, t, u):
        self.t = np.concatenate((self.t[1:], [t]))
        self.u = np.concatenate((self.u[1:], [u]))


def get_limits(vmin, vmax):
    Dv = vmax - vmin
    e = -np.log10(Dv)
    e = int(np.ceil(e))
    p = 10**e
    low = np.floor(vmin * p) / p
    up = np.ceil(vmax * p) / p
    return low, up


class Plotter:
    def __init__(self):
        self.lines = []
        self.signals = []
        self.axes = []

    def add_signal(self, axis, sig):
        line, = axis.plot(sig.t, sig.u)
        self.lines += [line]
        self.axes += [axis]
        self.signals += [sig]

    def update(self):
        if len(self.signals) == 0:
            return

        for l,sig in zip(self.lines, self.signals):
            l.set_data(sig.t, sig.u)

        # t diap
        t0 = np.max([sig.t[0] for sig in self.signals])
        t1 = np.max([sig.t[-1] for sig in self.signals])
        if t1 > t0:
            axis = self.axes[0]
            axis.set_xlim(t0, t1)
        
        # u diap
        for axis,sig in zip(self.axes, self.signals):
            u0 = np.min(sig.u)
            u1 = np.max(sig.u)
            if u1 > u0:
                low,up = get_limits(u0, u1)
                axis.set_ylim(low, up)

        plt.draw()


class JsonStream:

    def __init__(self, remoteip, remoteport, bufsz):
        remote = (remoteip, remoteport)
        self.bufsz = bufsz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(remote)
        self.data = b''
    
    def __del__(self):
        self.sock.close()

    def last(self):
        data = self.sock.recv(self.bufsz)
        if len(data) == 0:
            print('[err] conection closed')
            return None

        self.data += data
        i2 = self.data.rfind(b'\0')
        if i2 < 0:
            return ''

        i1 = self.data.rfind(b'\0', 0, i2)
        if i1 < 0:
            ans = self.data[0:i2]
            return ans

        ans = self.data[i1+1:i2]
        self.data = self.data[i1+1:]
        return ans


def parse_object(cfg):
    id = int(cfg['id'])
    p = np.array(cfg['p'], dtype='float')
    r = np.array(cfg['r'], dtype='float')
    return (id, p, r)


def parse_objects(cfg):
    t = int(cfg['ts']) * 1e-6
    objs = cfg['objects']
    return t, [parse_object(obj) for obj in objs]


def main():
    id1 = 1
    id2 = 3
    
    wndsz = 100
    _,(ax1,ax2,ax3) = plt.subplots(3, 1, True)
    ax1.grid(True)
    ax1.set_ylabel('x')
    ax2.grid(True)
    ax2.set_ylabel('y')
    ax3.grid(True)
    ax3.set_ylabel('z')
    plotter = Plotter()
    sig1 = None
    sig2 = None
    sig3 = None

    stream = JsonStream('127.0.0.1', 11011, 4096)

    while True:
        ans = stream.last()
        if ans is None:
            break

        if len(ans) == 0:
            continue

        cfg = json.loads(ans)
        t, objs = parse_objects(cfg)
        if len(objs) == 0:
            continue

        ids,ps,rs = zip(*objs)
        if id1 in ids and id2 in ids:
            i1 = ids.index(id1)
            i2 = ids.index(id2)
            p1 = ps[i1]
            p2 = ps[i2]
            r1 = rs[i1]
            r2 = rs[i2]
            T1 = maketransform(p1, r1)
            T2 = maketransform(p2, r2)
            dT = np.linalg.inv(T1).dot(T2)
            dp,dr = decomposetranmsform(dT)

            if sig1 is None:
                sig1 = Signal(t, dp[0])
                sig2 = Signal(t, dp[1])
                sig3 = Signal(t, dp[2])
                plotter.add_signal(ax1, sig1)
                plotter.add_signal(ax2, sig2)
                plotter.add_signal(ax3, sig3)
            else:
                sig1.append_measurement(t, dp[0])
                sig2.append_measurement(t, dp[1])
                sig3.append_measurement(t, dp[2])

            plotter.update()
            plt.pause(0.001)

    pass


if __name__ == '__main__':
    main()

