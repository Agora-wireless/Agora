#!/usr/bin/python3
import cv2
import socket
import struct
import pickle
from argparse import ArgumentParser
import random
import time

parser = ArgumentParser(description='stream video from Iris Client')
parser.add_argument('-d', '--device', type=int,
                    required=True, help='video device ID')
parser.add_argument('-i', '--ip', type=str,
                    default='localhost', help='UDP hostname')
parser.add_argument('-p', '--port', type=int,
                    required=True, help='UDP port number')
parser.add_argument('-s', '--size', type=int, default=152,
                    help="UDP packet size (bytes)")
args = parser.parse_args()

print('Connecting to socket {}:{}'.format(args.ip, args.port))

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect((args.ip, args.port))

cam = cv2.VideoCapture(args.device)
cam.set(3, 80)
cam.set(4, 60)

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

payload_size = struct.calcsize(">L")

random.seed(4)
HEADER = [random.randint(0, 255) for _ in range(args.size)]

img_counter = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cam.read()
    print("raw:", frame.shape)
    _, frame = cv2.imencode('.jpg', frame, encode_param)
    #data = bytearray(frame)
    #frame = cv2.resize(frame, (160, 120), interpolation=cv2.INTER_AREA)
    #print("downscale:", frame.shape)
    data = frame.tostring()
    size = len(data)

    # create and send start header
    start_header = bytearray(HEADER)
    print('sending', list(start_header))
    s.sendall(start_header)
    time.sleep(0.0025)

    print("{}: {}".format(img_counter, size))
    s.sendall(struct.pack(">L", size) + data[:args.size-payload_size])
    time.sleep(0.0025)
    data = data[args.size-payload_size:]
    for i in range(0, len(data), args.size):
        x = data[i:i+args.size]
        if len(x) < args.size:
            x += bytearray([0] * (args.size - len(x)))
        print('sending', list(x))
        s.sendall(x)
        time.sleep(0.0025)

    # create and send end header
    #end_header = bytearray([2 for _ in range(args.size)])
    # s.sendall(end_header)
    # time.sleep(0.001)

    img_counter += 1

cam.release()
