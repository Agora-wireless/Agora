import sys

n_servers = 3

if len(sys.argv) == 2:
    n_servers = int(sys.argv[1])

latency_data = []

for i in range(n_servers):
    f = open('frame_latency_{0}.txt'.format(i), 'r')
    lines = f.readlines()
    n_lines = len(lines)
    for j in range(n_lines):
        words = lines[j].split()
        if len(latency_data) <= j:
            latency_data.append([int(words[0]), int(words[1]), int(words[2]), int(words[3]), int(words[4]), int(words[5]), int(words[6])])
        else:
            tp = latency_data[j]
            if tp[1] > int(words[1]):
                tp[1] = int(words[1])
            if tp[2] < int(words[2]):
                tp[2] = int(words[2])
            if tp[3] < int(words[3]):
                tp[3] = int(words[3])
            if tp[4] < int(words[4]):
                tp[4] = int(words[4])
            if tp[5] < int(words[5]):
                tp[5] = int(words[5])
            if tp[6] < int(words[6]):
                tp[6] = int(words[6])
            latency_data[j] = tp
    f.close()

out_f = open('frame_latency_all.txt', 'w')

for frame in latency_data:
    print >>out_f, frame[0], (frame[2]-frame[1])/1000.0, (frame[3]-frame[2])/1000.0, (frame[4]-frame[3])/1000.0, (frame[5]-frame[4])/1000.0, (frame[6]-frame[5])/1000.0, (frame[6]-frame[1])/1000.0

out_f.close()