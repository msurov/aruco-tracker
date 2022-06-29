import socket
import select
import json
import numpy as np
import matplotlib.pyplot as plt


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


class StreamParser:
    def __init__(self, ip, port):
        remote = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(remote)
        self.fill_size = 16384
        self.buffer = b''
    

    def __read_all(self):
        data = b''
        while True:
            ready = select.select([self.sock], [], [], 0)
            if ready[0]:
                ans = self.sock.recv(self.fill_size)
                if len(ans) == 0:
                    return -1
                data += ans
            else:
                break
        return data
    
    def __fill_buffer(self):
        __read_all

        fill_size = 16384
        data = sock.recv(fill_size)
        if len(data) == 0:
            return False
        self.buffer += data

        while len(data) == fill_size:
            data = sock.recv(fill_size)
            self.buffer += data
        return True
    
    def __take_newest(self)
        
        if len(data) == 0:
            print('[err] conection closed. Reconnecting...')
            #self.sock.connect(self._remote)
            return None

        self.data += data
        i2 = self.data.rfind(b'\0')
        if i2 < 0:
            return None

        i1 = self.data.rfind(b'\0', 0, i2)
        if i1 < 0:
            ans = self.data[0:i2]
            return ans

        ans = self.data[i1 + 1:i2]
        self.data = self.data[i1 + 1:]
        return ans

    def newest(self):
        ok = self.__fill_buffer()
        if not ok:
            raise Exception('Connection closed')
        newest = self.__take_newest()


def main():
    remoteip = '127.0.0.1'
    remoteport = 11011
    

    wndsz = 20
    signals = {}
    plotter = Plotter(wndsz)

    while True:
        data = sock.recv(4*4096)
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
