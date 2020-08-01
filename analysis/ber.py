#!/usr/bin/python3
import numpy as np
from argparse import ArgumentParser
import csv

# parse arguments
parser = ArgumentParser(description="compute BER on MAC data offline")
parser.add_argument('-u', '--user', type=str,
                    default='macuser.log', help='path to macuser.log')
parser.add_argument('-b', '--bs', type=str, default='macbs.log',
                    help='path to macbs.log')
parser.add_argument('-s', '--size', type=int, default=98,
                    help='size of udp datagram (bytes)')
args = parser.parse_args()


def parse_log(path):
    f = open(path, 'r')
    data = []
    merge = ''
    for line in f:
        if 'Received data from app' in line:
            # for macuser logs
            continue
        elif 'Sent data for frame' in line:
            # macbs logs
            continue

        x = line.split()
        assert len(x) == args.size, "found {} bytes, expected {} bytes: {}".format(
            len(x), args.size, line)

        x = ''.join([format(int(d), '08b') for d in x])

        data += [x]

    return data


def find_match(user, bs):
    for i, x in enumerate(bs):
        if x in user:
            j = user.index(x)
            print('found match at bs={}, user={}'.format(i, j))
            return j, i
    return None, None


def compute_ber(user, bs):
    total_bits = 0
    error_bits = 0
    perfect = []

    for i, (u, b) in enumerate(zip(user, bs)):
        # check same number of bits
        assert len(u) == len(b), "u = {}, b = {}; {}, {}".format(
            len(u), len(b), u, b)

        error = sum(int(x) ^ int(y) for x, y in zip(list(u), list(b)))
        if error == 0:
            perfect += [i]

        error_bits += error
        total_bits += len(b)

    print("{}/{} perfect Millipede frames".format(len(perfect), min(len(user), len(bs))))
    return error_bits/total_bits


# parse user, bs logs
user = parse_log(args.user)
bs = parse_log(args.bs)
print('Parsed {} frames from user, {} frames from bs'.format(len(user), len(bs)))

# find match
user_start, bs_start = find_match(user, bs)
user = user[user_start:]
bs = bs[bs_start:]

# compute ber
ber = compute_ber(user, bs)
print("ber = {}".format(ber))
