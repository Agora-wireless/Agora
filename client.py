#!/usr/bin/python3

import subprocess as sh
import time
from argparse import ArgumentParser

MP = './build/'

cmd_user = MP + '/user data/ue-one-ul-hw.json'
cmd_macuser = MP + '/macuser 2 2420 data/ue-one-ul-hw.json'

# parse arguments
parser = ArgumentParser(description="launch hw client")
parser.add_argument('-a', '--app', type=str,
                    required=True, help='client app traffic')
args = parser.parse_args()

if args.app == "rand":
    cmd_app = './src/app/stream_rand.py -p {p} -s 152'
elif args.app == "video":
    #cmd_app = './src/app/stream_video.py -p {p}'
    cmd_app = './src/app/video_client.py -p {p} -s 49 -d 2'
elif args.app == "none":
    pass
else:
    print('App type {} not supported.'.format(args.app))

# start macuser
print('Starting macuser...')
logm = open('macuser.log', 'w')
pm = sh.Popen(cmd_macuser.split(), stdout=logm)

# start client apps
if args.app != "none":
    print('Starting client apps...')
    loga0 = open('app0.log', 'w')
    pa0 = sh.Popen(cmd_app.format(p=8090).split(), stdout=loga0)

# start user
print('Starting user...')
logu = open('user.log', 'w')
pu = sh.Popen(cmd_user.split(), stdout=logu)

# wait
print('Waiting...')
pu.wait()
pm.wait()
print('...done!')
