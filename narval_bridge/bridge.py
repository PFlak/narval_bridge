import re
import socket
import random
import time

class Bridge:
    def __init__(self, ip: str, port: int):
        self.__ip = None
        self.__port = None
        self.__socket = None

        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @property
    def ip(self):
        return self.__ip
    
    @ip.setter
    def ip(self, ip: str):
        if not re.fullmatch('^((25[0-5]|(2[0-4]|1[0-9]|[1-9]|)[0-9])(\.(?!$)|$)){4}$', ip):
            raise Exception(f"Wrong ip address")

        self.__ip = ip

        if not self.port:
            return

    @property
    def port(self):
        return self.__port
    
    @port.setter
    def port(self, port):
        self.__port = port

    @property
    def sock(self) -> socket.socket:
        return self.__socket
    
    @sock.setter
    def sock(self, socket: socket.socket):
        self.__socket = socket

    def send(self, data):
        data = bytes(data, 'utf-8')
        self.sock.sendto(data, (self.ip, self.port))

if __name__ == '__main__':

    bridge = Bridge('10.10.10.10', 5000)

    while True:
        data = random.randint(1, 100)
        data = f"SOMEDATA {data}"

        bridge.send(data=data)
        time.sleep(1)
        
