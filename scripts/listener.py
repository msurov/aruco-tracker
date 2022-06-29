import socket
import json
import numpy as np
import matplotlib.pyplot as plt


class Signal:
    def __init__(self, t0, u0, wndsz=20):
        self.t = np.array([t0] * wndsz)
        self.u = np.array([u0] * wndsz)

    def append_measurement(self, t, u):
        self.t = np.concatenate((self.t[1:], [t]))
        self.u = np.concatenate((self.u[1:], [u]))

class Plotter:
    def __init__(self, wndsz):
        self.lines = []
        self.signals = []
        self.wndsz = wndsz
        plt.axis([0, wndsz, 0, 1])

    def add_signal(self, s):
        line, = plt.plot(s.t, s.u)
        self.lines += [line]
        self.signals += [s]

    def update(self):
        if len(self.signals) == 0:
            return

        for l,s in zip(self.lines, self.signals):
            print('updating', s.t, s.u)
            l.set_data(s.t, s.u)

        t0 = np.max([s.t[0] for s in self.signals])
        t1 = np.max([s.t[-1] for s in self.signals])
        ax = plt.gca()
        ax.set_xlim(t0, t1)
        plt.draw()


def parse_object(cfg):
    id = int(cfg['id'])
    p = np.array(cfg['p'], dtype='float')
    r = np.array(cfg['r'], dtype='float')
    return (id, p, r)

def parse_objects(cfg):
    t = int(cfg['ts']) * 1e-6
    objs = cfg['objects']
    return t, [parse_object(obj) for obj in objs]

def take_newest(data):
    i = data.rfind(b'\x00', 0, -1)
    if i < 0:
        s = data[:]
        s = s.strip(b'\n\t \x00')
        s = s.decode('utf-8')
        return s
    s = data[i+1:]
    s = s.strip(b'\n\t \x00')
    s = s.decode('utf-8')
    s = s.strip()
    return s

def main():
    remoteip = '127.0.0.1'
    remoteport = 11011
    remote = (remoteip, remoteport)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(remote)

    wndsz = 20
    signals = {}
    plotter = Plotter(wndsz)

    while True:
        data = sock.recv(4096)
        if len(data) == 0:
            print('conection closed')
            break

        newest = take_newest(data)
        print('1: ', newest, len(newest))
        cfg = json.loads(newest)
        print('2: ', cfg)
        t, objs = parse_objects(cfg)
        for (id,p,r) in objs:
            u = p[0]
            if id in signals:
                signals[id].append_measurement(t, u)
            else:
                sig = Signal(t, u)
                signals[id] = sig
                plotter.add_signal(sig)

        plotter.update()
        plt.pause(0.001)

    sock.close()

if __name__ == '__main__':
    main()
