#!/usr/bin/python3

# BS-side application that receives data packets from Agora MAC
# packet_size argument should correspond to the number of bits per
# PHY layer frame in Agora
# Running example: ./python/bs_app.py --packet-size 66 --stream-num 1

import socket
import sys
import time
from optparse import OptionParser
import signal


def bs_datarecv_app(size, streams, base_port):
    # Create a UDP socket
    sock = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            for i in range(streams)]
    server_address = [('localhost', base_port + i) for i in range(streams)]
    for i in range(streams):
        sock[i].bind(server_address[i])

    signal.signal(signal.SIGINT, signal_handler)
    while(True):
        for i in range(streams):
            message, server = sock[i].recvfrom(size)
            print("Stream %d: " % i + "{}".format(list(message)))


def signal_handler(signal, frame):
    sys.exit(0)


def main():
    parser = OptionParser()
    parser.add_option("--stream-num", type="int", dest="stream_num",
                      help="number of user spatial streams", default=1)
    parser.add_option("--packet-size", type="int", dest="packet_size",
                      help="size of packets in bytes", default=66)
    parser.add_option("--base-port", type="int", dest="base_port",
                      help="server base listening port number to transmit data", default=8080)
    (options, args) = parser.parse_args()

    bs_datarecv_app(options.packet_size, options.stream_num, options.base_port)


if __name__ == '__main__':
    main()
