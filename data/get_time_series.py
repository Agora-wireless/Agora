import json
import math

def parse_time(time_str):
    tokens = time_str.split(':')
    return int(tokens[0]) * 3600 + int(tokens[1]) * 60 + int(tokens[2])

input = open("tddconfig-sim-ul-distributed.json", "r")
json_struct = json.load(input)
input.close()
core_offset = json_struct['core_offset']
subcarrier_num_list = json_struct['subcarrier_num_list']
subcarrier_block_size = json_struct['subcarrier_block_size']
decode_thread_num = json_struct['decode_thread_num']
socket_thread_num = json_struct['socket_thread_num']

server_num = len(json_struct['bs_server_addr_list'])

usage_table = {}

time_start = 0
time_end = 9999999999

core_base = 0
for i in range(server_num):
    input = open("cpu_usage_%d.txt" % i, "r")
    lines = input.readlines()[2:]
    input.close()
    n_record = len(lines) / 34
    core_start = core_offset + 1
    core_end = core_start + socket_thread_num + 1 + (subcarrier_num_list[i] + subcarrier_block_size - 1) / subcarrier_block_size + decode_thread_num[i]
    for j in range(core_base, core_base + core_end - core_start):
        usage_table[j] = {}
    time_start_tmp = 999999999
    time_end_tmp = 0
    for j in range(n_record):
        time = parse_time(lines[j * 34 + 1].split()[0])
        time_start_tmp = min(time, time_start_tmp)
        time_end_tmp = max(time, time_end_tmp)
        for k in range(core_start, core_end):
            line_idx = j * 34 + 1 + k
            tokens = lines[line_idx].split()
            usage = 100.0 - float(tokens[-1])
            usage_table[core_base + k - core_start][time] = usage
    core_base += core_end - core_start
    time_start = max(time_start, time_start_tmp)
    time_end = min(time_end, time_end_tmp)

usage_sum = {}

for i in range(time_start, time_end):
    s = 0.0
    for j in range(core_base):
        s += usage_table[j][i]
    s /= core_base
    usage_sum[i] = s

print usage_sum