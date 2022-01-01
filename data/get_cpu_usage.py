import json

input = open("tddconfig-sim-ul-distributed.json", "r")
json_struct = json.load(input)
input.close()
core_offset = json_struct['core_offset']
subcarrier_num_list = json_struct['subcarrier_num_list']
subcarrier_block_size = json_struct['subcarrier_block_size']
decode_thread_num = json_struct['decode_thread_num']
socket_thread_num = json_struct['socket_thread_num']

server_num = len(json_struct['bs_server_addr_list'])

usage_table = []

for i in range(server_num):
    input = open("cpu_usage_%d.txt" % i, "r")
    lines = input.readlines()
    input.close()
    n_record = len(lines) / 34
    core_start = core_offset + 1
    core_end = core_start + socket_thread_num + 1 + (subcarrier_num_list[i] + subcarrier_block_size - 1) / subcarrier_block_size + decode_thread_num[i]
    usage_list = [0] * (core_end - core_start)
    for j in range(n_record):
        for k in range(core_start, core_end):
            line_idx = j * 34 + 1 + k
            tokens = lines[line_idx].split()
            usage = 100.0 - float(tokens[-1])
            usage_list[k - core_start] += usage
    for k in range(core_start, core_end):
        usage_list[k - core_start] /= n_record
        usage_table.append(usage_list[k - core_start])

s = 0.0
for i in range(len(usage_table)):
    s += usage_table[i]
s /= len(usage_table)
print s