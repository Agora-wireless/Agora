import json

output = open("measurement/fig_9999_latency_core.txt", "w")

output.write("Data 64*8 worked 64*16 worked 64*32 worked 128*16 worked 128*32 worked 150*32 worked\n")

settings = [(64, 8), (64, 16), (64, 32), (128, 16), (128, 32), (150, 32)]

latency_res = {}
worked_res = {}

for i in range(6):
    ant = settings[i][0]
    ue = settings[i][1]
    for c in range(20, 100, 10):
        latencies = []
        for t in range(10):
            input = open("frame_latency/frame_latency_all_{}_{}_1ms_{}c_{}.txt".format(ant, ue, c, t), "r")
            lines = input.readlines()[1000:]
            for line in lines:
                tokens = line.split()
                latency = float(tokens[-1])
                latencies.append(latency)
            input.close()
        latencies.sort()
        latency_res[(ant, ue, c)] = latencies[int(len(latencies) * 0.9999)]
        input = open("config/tddconfig-sim-ul-distributed-{}-{}-1ms-{}c.json".format(ant, ue, c), "r")
        json_struct = json.load(input)
        if json_struct.has_key('slot_size'):
            worked_res[(ant, ue, c)] = 0
        else:
            worked_res[(ant, ue, c)] = 1
        input.close()

for c in range(20, 100, 10):
    output.write("%d %f %d %f %d %f %d %f %d %f %d %f %d\n" % (c, latency_res[(64, 8, c)], worked_res[(64, 8, c)], latency_res[(64, 16, c)], worked_res[(64, 16, c)], latency_res[(64, 32, c)], worked_res[(64, 32, c)], latency_res[(128, 16, c)], worked_res[(128, 16, c)], latency_res[(128, 32, c)], worked_res[(128, 32, c)], latency_res[(150, 32, c)], worked_res[(150, 32, c)]))
output.close()