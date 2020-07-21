#!/usr/bin/python3

import subprocess as sh
import time
from argparse import ArgumentParser

MP = './build/'

# parse arguments
parser = ArgumentParser(description="launch Millipede base station (hw)")
parser.add_argument('-n', '--num', type=int, default=1,
                    help='number of Iris clients talking to base station')
args = parser.parse_args()

# setup cmds
cfg = 'data/bs-ul-hw.json' if args.num == 2 else 'data/bs-one-ul-hw.json'
cmd_millipede = MP + '/millipede ' + cfg

# start millipede
print('Starting millipede...')
logm = open('millipede.log', 'w')
pm = sh.Popen(cmd_millipede.split(), stdout=logm)

time.sleep(1)

# wait
print('Waiting...')
pm.wait()
print('...done!')
