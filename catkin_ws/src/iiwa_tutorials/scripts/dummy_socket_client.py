#!/usr/bin/env python3

import socket
import time
from datetime import datetime

HOST_IP = "127.0.0.1"
PORT = 12345  # port number
DATE_SIZE = 1024  # data bytes to receive
INTERVAL = 3  # wait time until nect connection start
RETRY_TIMES = 5  # number of retry when socket connection


class SocketClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None

    def connect(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        for x in range(RETRY_TIMES):
            try:
                client_socket.connect((self.host, self.port))  # connection with server
                self.socket = client_socket
                print('[{0}] server connect -> address : {1}:{2}'.format(
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    self.host, self.port))
                break
            except socket.error:
                print('[{0}] retry after wait{1}s'.format(
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'), str(INTERVAL)))
                time.sleep(INTERVAL)

    def send(self):
        input_data = input("send data:")
        print('[{0}] input data : {1}'.format(
            datetime.now().strftime('%Y-%m-%d %H:%M:%S'), input_data))
        input_data = input_data.encode('utf-8')
        self.socket.send(input_data)

    def recv(self):
        rcv_data = self.socket.recv(DATE_SIZE)
        rcv_data = rcv_data.decode('utf-8')
        print('[{0}] recv data : {1}'.format(
            datetime.now().strftime('%Y-%m-%d %H:%M:%S'), rcv_data))
        return rcv_data

    def send_rcv(self):
        self.send()
        return self.recv()

    def close(self):
        self.socket.close()
        self.socket = None


if __name__ == '__main__':
    client = SocketClient(HOST_IP, PORT)
    client.connect()

    while True:
        if client.socket is not None:
            if client.send_rcv() == 'quit':
                client.close()
        else:
            break
