#!/usr/bin/python3
import subprocess as sh
from argparse import ArgumentParser

parser = ArgumentParser(description='run video upload client with ffmpeg')
parser.add_argument('-d', '--device', type=int,
                    required=True, help='video device ID')
parser.add_argument('-i', '--ip', type=str,
                    default='localhost', help='UDP hostname')
parser.add_argument('-p', '--port', type=int,
                    required=True, help='UDP port number')
parser.add_argument('-s', '--size', type=int, default=152,
                    help='UDP packet size (bytes)')
parser.add_argument('-b', '--bitrate', type=int,
                    default=302500, help='UDP constant bitrate')
parser.add_argument('-f', '--fps', type=int,
                    default=1, help='frame rate (fps)')
parser.add_argument('-r', '--resolution', type=str,
                    default='80x60', help='frame size')
args = parser.parse_args()

cmd = 'sudo ffmpeg -r {fps} \
    -i /dev/video{device} \
    -s {res} -flush_packets 0 \
    -b:v {bitrate} -minrate {bitrate} -maxrate {bitrate} \
    -f mpegts \
    udp://{host}:{port}?pkt_size={size}?bitrate={bitrate} \
    -s {res} -flush_packets 0 \
    -b:v {bitrate} -minrate {bitrate} -maxrate {bitrate} \
    -f mpegts \
    udp://{host}:9090?pkt_size={size}?bitrate={bitrate}'


ffmpeg = cmd.format(fps=args.fps, device=args.device, res=args.resolution,
                    host=args.ip, port=args.port, size=args.size, bitrate=args.bitrate)
print('Launching: {}'.format(ffmpeg))
sh.run(ffmpeg.split())
