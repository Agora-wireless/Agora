import socket
import sys
import time
import random
from optparse import OptionParser
import signal

def bs_datagen_app(dly, size):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('localhost', 8070)

    signal.signal(signal.SIGINT, signal_handler)
    while(True):
        message = bytearray(random.getrandbits(8) for _ in xrange(size))
        print("{}".format(list(message)))
        sock.sendto(message, server_address)
        time.sleep(dly)

def signal_handler(signal, frame):
    sys.exit(0)

def main():
    parser = OptionParser()
    parser.add_option("--delay", type="float", dest="delay", help="", default=0.00242)
    parser.add_option("--packet-size", type="int", dest="packet_size", help="size of packets in bytes", default=66)
    (options, args) = parser.parse_args()

    bs_datagen_app(options.delay, options.packet_size)

if __name__ == '__main__':
    main()
