#!/usr/bin/python3

# BS-side application that receives data packets from Millepede MAC
# packet_size argument should correspond to the number of bits per
# PHY layer frame in Millipede
# Running example: ./python/bs_app.py --packet-size 66

import socket
import sys
import time
from optparse import OptionParser
import signal


def bs_datagen_app(size):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('127.0.0.1', 8080)
    sock.bind(server_address)

    signal.signal(signal.SIGINT, signal_handler)
    while(True):
        message, server = sock.recvfrom(size)
        print("{}".format(list(message)))


def signal_handler(signal, frame):
    sys.exit(0)


def main():
    parser = OptionParser()
    parser.add_option("--packet-size", type="int", dest="packet_size",
                      help="size of packets in bytes", default=66)
    (options, args) = parser.parse_args()

    bs_datagen_app(options.packet_size)


if __name__ == '__main__':
    main()
