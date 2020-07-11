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
cmd_macbs = MP + '/macbs 2 0 ' + cfg

# start millipede
print('Starting millipede...')
logm = open('millipede.log', 'w')
pm = sh.Popen(cmd_millipede.split(), stdout=logm)

time.sleep(1)

# start macbs
print('Starting macbs...')
logb = open('macbs.log', 'w')
pb = sh.Popen(cmd_macbs.split(), stdout=logb)

# wait
print('Waiting...')
pm.wait()
pb.wait()
print('...done!')
