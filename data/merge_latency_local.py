import sys

n_servers = 3

if len(sys.argv) == 2:
    n_frames = int(sys.argv[1])

latency_data = []

f = open('frame_latency.txt', 'r')
lines = f.readlines()
n_lines = len(lines)
for j in range(n_lines):
    words = lines[j].split()
    latency_data.append([int(words[0]), int(words[1]), int(words[2]), int(words[3]), int(words[4]), int(words[5])])
f.close()

out_f = open('frame_latency_local.txt', 'w')

for frame in latency_data:
    print >>out_f, frame[0], (frame[2]-frame[1])/1000.0, (frame[3]-frame[2])/1000.0, (frame[4]-frame[3])/1000.0, (frame[5]-frame[4])/1000.0, (frame[5]-frame[1])/1000.0

out_f.close()