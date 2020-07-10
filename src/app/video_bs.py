#!/usr/bin/python3

import cv2
import socket
import struct
import pickle
import random
import sys
from argparse import ArgumentParser
import numpy as np

parser = ArgumentParser(description='playback video from Iris Clients')
parser.add_argument('-i', '--ip', type=str,
                    default='', help='UDP hostname')
parser.add_argument('-p', '--port', type=int,
                    required=True, help='UDP port number')
args = parser.parse_args()

conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
conn.bind((args.ip, args.port))
print('Listening on {}:{}'.format(args.ip, args.port))

random.seed(4)
HEADER = [random.randint(0, 255) for _ in range(49)]

data = b""
payload_size = struct.calcsize(">L")
num_frames = 0
in_frame = False
while True:

    while not in_frame:
        header = conn.recv(1024)
        header = list(header)
        if HEADER == header:
            print("header:", header)
            in_frame = True

    # start processing frame
    num_frames += 1
    print('Received header, frame {}'.format(num_frames))
    # while len(data) < payload_size:
    #    data += conn.recv(1024)

    #packed_msg_size = data[:payload_size]
    #data = data[payload_size:]
    #msg_size = struct.unpack(">L", packed_msg_size)[0]
    msg_size = 57600
    print("frame size: {}".format(msg_size))

    # if msg_size > 50000:
    #    print('frame too large')
    #    in_frame = False
    #    d = b""
    #    continue

    while len(data) < msg_size:
        data += conn.recv(1024)
    frame_data = data[:msg_size]

    try:
        frame = np.fromstring(frame_data, dtype='uint8')
        #frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        print('frame dim:', frame.shape)
        frame = frame.reshape((120, 160, 3))
        cv2.imshow('ImageWindow', frame)
        cv2.waitKey(1)
    except Exception as e:
        print('Error rendering frame: ', e)

    in_frame = False
    data = b""
