import json

output = open("measurement/fig_fft_latency.txt", "w")

output.write("Data Total IQ FFT Subcarrier Decode\n")

settings = [(128, 16)]
cores = [59]

for i in range(1):
    ant = settings[i][0]
    ue = settings[i][1]
    c = cores[i]
    latencies = []
    for t in range(10):
        input = open("frame_latency/frame_latency_all_{}_{}_1ms_{}c_{}.txt".format(ant, ue, c, t), "r")
        lines = input.readlines()[1000:]
        for line in lines:
            tokens = line.split()
            latency = float(tokens[-1])
            latencies.append((latency, line))
        input.close()
    latencies.sort()
    idx = int(len(latencies) * 0.9999)
    line = latencies[idx][1]
    tokens = line.split()
    output.write("Hydra %f %f %f %f %f\n" % (latencies[idx][0], float(tokens[1]), 0, float(tokens[2]), float(tokens[3]) + float(tokens[4])))

    latencies = []
    for t in range(10):
        input = open("frame_latency/frame_latency_all_{}_{}_1ms_fft_{}.txt".format(ant, ue, t), "r")
        lines = input.readlines()[1000:]
        for line in lines:
            tokens = line.split()
            latency = float(tokens[-1])
            latencies.append((latency, line))
        input.close()
    latencies.sort()
    idx = int(len(latencies) * 0.9999)
    line = latencies[idx][1]
    tokens = line.split()
    output.write("FFT %f %f %f %f %f\n" % (latencies[idx][0], float(tokens[1]), float(tokens[2]), float(tokens[3]), float(tokens[4]) + float(tokens[5])))

output.close()