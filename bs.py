#!/usr/bin/python3

import subprocess as sh
import time

MP = './build/'

cmd_millipede = MP + '/millipede data/bs-one-ul-hw.json'
cmd_macbs = MP + '/macbs 2 0 data/bs-one-ul-hw.json'

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
