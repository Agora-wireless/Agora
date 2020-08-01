#!/usr/bin/python3
import socket
import time
import random
from argparse import ArgumentParser

parser = ArgumentParser(description='send random data over UDP')
parser.add_argument('-i', '--ip', type=str,
                    default='localhost', help='UDP hostname')
parser.add_argument('-p', '--port', type=int,
                    required=True, help='UDP port number')
parser.add_argument('-s', '--size', type=int, default=152,
                    help='UDP packet size (bytes)')
args = parser.parse_args()

# open socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect((args.ip, args.port))

while(True):
    # create random data
    data = bytearray([random.randint(0, 255) for _ in range(args.size)])

    # send data
    s.sendall(data)

    # sleep for 1ms
    time.sleep(0.001)
