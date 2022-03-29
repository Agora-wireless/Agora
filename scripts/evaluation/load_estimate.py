import sys
import os

ant_array = [32, 32, 64, 64, 128, 128, 256]
ue_array = [8, 16, 16, 32, 16, 32, 32]
sc_num = 1216
sym_num = 14
ul_sym_num = 13
dl_sym_num = 13
mode = "uplink"

if len(sys.argv) != 3:
    print "Invalid arguments!"

fn = sys.argv[1]
file_input = open(fn, "r")
mode = sys.argv[2]

lines = file_input.readlines()
file_input.close()

if mode == "uplink":
    for i in range(len(ant_array)):
        tokens = lines[i].split()
        csi_ns = float(tokens[0])
        zf_us = float(tokens[1])
        demul_ns = float(tokens[2])
        decode_us = float(tokens[3])
        encode_us = float(tokens[4])
        precode_ns = float(tokens[5])
        sc_thread_load = csi_ns * sc_num * sym_num + zf_us * 1000.0 * (sc_num / ue_array[i]) + demul_ns * sc_num * ul_sym_num
        decode_thread_load = decode_us * 1000.0 * ue_array[i] * ul_sym_num
        print("{} {}".format(sc_thread_load, decode_thread_load))

# csi * sc_num * sym_num + zf * (sc_num / ue_num) + demul * sc_num * ul_sym_num
