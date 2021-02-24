#!/usr/bin/python3

# Client-side application that generates random bit data packets
# and sends through a socket to Agora client MAC for transmission
# through PHY layer. packet_size argument should correspond to the
# number of bits per PHY layer frame in Agora. delay argument
# specifies the delay between transmission of two consecutive packets.
# Running example: ./python/client_app.py --delay 0.00242 --packet-size 66

import socket
import sys
import time
import random
from optparse import OptionParser
import signal


def client_datagen_app(dly, size, streams, base_port):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('localhost', base_port)

    signal.signal(signal.SIGINT, signal_handler)
    while(True):
        for u in range(streams):
            message = bytearray(random.getrandbits(8) for _ in range(size))
            print("Stream %d: " % u + "{}".format(list(message)))
            sock.sendto(message, server_address)
        time.sleep(dly)


def signal_handler(signal, frame):
    sys.exit(0)


def main():
    parser = OptionParser()
    parser.add_option("--delay", type="float", dest="delay",
                      help="", default=0.00242)
    parser.add_option("--packet-size", type="int", dest="packet_size",
                      help="size of packets in bytes", default=66)
    parser.add_option("--stream-num", type="int", dest="stream_num",
                      help="number of user spatial streams", default=1)
    parser.add_option("--base-port", type="int", dest="base_port",
                      help="server base listening port number to transmit data", default=9070)
    (options, args) = parser.parse_args()

    client_datagen_app(options.delay, options.packet_size,
                       options.stream_num, options.base_port)


if __name__ == '__main__':
    main()
