import sys

n_servers = 3
parent_dir = "."

if len(sys.argv) == 2:
    n_servers = int(sys.argv[1])
elif len(sys.argv) == 3:
    n_servers = int(sys.argv[1])
    parent_dir = sys.argv[2]

latency_data = []

for i in range(n_servers):
    f = open('{0}/frame_latency_{1}.txt'.format(parent_dir, i), 'r')
    lines = f.readlines()
    n_lines = len(lines)
    for j in range(n_lines):
        words = lines[j].split()
        if len(latency_data) <= j:
            latency_data.append([int(words[0]), int(words[1]), int(words[2]), int(words[3]), int(words[4]), int(words[5])])
        else:
            tp = latency_data[j]
            diff = tp[1] - int(words[1])
            if tp[2] < int(words[2]) + diff:
                tp[2] = int(words[2]) + diff
            if tp[3] < int(words[3]) + diff:
                tp[3] = int(words[3]) + diff
            if tp[4] > int(words[4]) + diff:
                tp[4] = int(words[4]) + diff
            if tp[5] < int(words[5]) + diff:
                tp[5] = int(words[5]) + diff
            latency_data[j] = tp
    f.close()

out_f = open('{0}/frame_latency_all.txt'.format(parent_dir), 'w')

for frame in latency_data:
    print >>out_f, frame[0], (frame[2]-frame[1])/1000.0, (frame[3]-frame[2])/1000.0, (frame[4]-frame[3])/1000.0, (frame[5]-frame[4])/1000.0, (frame[5]-frame[1])/1000.0

out_f.close()