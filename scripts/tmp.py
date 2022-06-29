import socket
import select


class StreamParser:
    def __init__(self, ip, port):
        remote = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(remote)
        self.fill_size = 16384
        self.buffer = b''

    def read_all(self):
        data = b''
        while True:
            ready = select.select([self.sock], [], [], 0)
            if len(ready[0]) > 0:
                ans = self.sock.recv(self.fill_size)
                if len(ans) == 0:
                    return None
                data += ans
                if len(ans) < self.fill_size:
                    break
            else:
                break
        return data

if __name__ == '__main__':
    parser = StreamParser('localhost', 11011)
    while True:
        data = parser.read_all()
        if len(data) > 0:
            print(len(data))
