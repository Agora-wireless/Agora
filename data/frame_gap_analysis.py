for i in range(12):
    f = open("frame_latency_{}.txt".format(i), "r")
    lines = f.readlines()[1000:]
    max_gap = 0
    max_gap_frame = -1
    for j in range(len(lines)):
        t1 = int(lines[j].split()[1])
        t2 = int(lines[j].split()[2])
        gap = t2 - t1
        if gap > max_gap:
            max_gap = gap
            max_gap_frame = j + 1000
    print("Max gap for file {} is {}, frame {}".format(i, max_gap, max_gap_frame))
    f.close()