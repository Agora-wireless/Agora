#!/usr/bin/python3

import subprocess as sh
import time
from argparse import ArgumentParser

MP = './build/'

# parse arguments
parser = ArgumentParser(description="launch hw client")
parser.add_argument('-a', '--app', type=str,
                    required=True, help='client app traffic')
parser.add_argument('-n', '--num', type=int, default=1,
                    help='number of Iris clients')
args = parser.parse_args()

if args.app == "rand":
    cmd_app = './src/app/stream_rand.py -p {p} -s 98'
elif args.app == "video":
    cmd_app = './src/app/video_client.py -p {p} -s 98 -d {d}'
elif args.app == "none":
    pass
else:
    print('App type {} not supported.'.format(args.app))

# setup cmds
cfg = 'data/ue-ul-hw.json' if args.num == 2 else 'data/ue-one-ul-hw.json'
cmd_user = MP + '/user ' + cfg

# start user
print('Starting user...')
logu = open('user.log', 'w')
pu = sh.Popen(cmd_user.split(), stdout=logu)

time.sleep(5)

# start client apps
if args.app != "none":
    loga = []
    proca = []
    for i in range(args.num):
        print('Starting client {}...'.format(i))
        loga += [open('app{}.log', 'w')]
        cmda = cmd_app.format(p=8070+i, d=i*2)
        proca += [sh.Popen(cmda.split(), stdout=loga[i])]

# wait
print('Waiting...')
pu.wait()
print('...done!')
