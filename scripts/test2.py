import matplotlib.pyplot as plt
import numpy as np


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
        for l,s in zip(self.lines, self.signals):
            print('updating', s.t, s.u)
            l.set_data(s.t, s.u)

        t0 = np.max([s.t[0] for s in self.signals])
        t1 = np.max([s.t[-1] for s in self.signals])
        ax = plt.gca()
        ax.set_xlim(t0, t1)
        plt.draw()

wndsz = 20
s1 = Signal(0, 0, wndsz)
p = Plotter(wndsz)
p.add_signal(s1)

for i in range(100):
    s1.append_measurement(i/100, 1. / (i + 1))
    p.update()
    
    plt.pause(0.03)

plt.show()
