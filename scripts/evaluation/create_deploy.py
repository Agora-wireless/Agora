import json
import sys

app_node_list = ["node9", "node10", "node11", "node12", "node13", "node14", "node15", "node16", "node17", "node18", "node19", "node20", "node21", "node22", "node24"]
server_num_min = 6

def ceil_divide(a, b):
    s = 0
    if a % b > 0:
        s = 1
    s += a / b
    return s

target_core_num = int(sys.argv[1])
deploy_fn = sys.argv[2]
template_fn = sys.argv[3]

template_input = open(template_fn, "r")
template_json = json.load(template_input)
template_input.close()
rx_thread_num = template_json['rx_thread_num']
antenna_num = template_json['antenna_num']
ue_num = template_json['ue_num']
core_offset = template_json['core_offset']
phy_core_num = template_json['phy_core_num']
ofdm_data_num = template_json['ofdm_data_num']

deploy_input = open(deploy_fn, "r")
deploy_json = json.load(deploy_input)
deploy_input.close()
subcarrier_num_list = deploy_json['subcarrier_num_list']
subcarrier_block_list = deploy_json['subcarrier_block_list']
coding_thread_num = deploy_json['coding_thread_num']
server_num = len(subcarrier_num_list)

subcarrier_core_num = 0
for i in range(server_num):
    subcarrier_core_num += ceil_divide(subcarrier_num_list[i], subcarrier_block_list[i])
    
coding_core_num = 0
for i in range(server_num):
    coding_core_num += coding_thread_num[i]

core_num_each_server = phy_core_num - core_offset - 1
num_server = ceil_divide(target_core_num, core_num_each_server)
if num_server < server_num_min:
    num_server = server_num_min

core_left_for_proc = (target_core_num - rx_thread_num * num_server) * 2 - num_server
target_core_num_for_subcarrier = core_left_for_proc * subcarrier_core_num / (subcarrier_core_num + coding_core_num) + 1

subcarrier_block_total_num = ofdm_data_num / ue_num
base_block_num_per_server = subcarrier_block_total_num / num_server
super_server_num = subcarrier_block_total_num % num_server
new_subcarrier_num_list = []
for i in range(num_server - super_server_num):
    new_subcarrier_num_list.append(base_block_num_per_server * ue_num)
for i in range(super_server_num):
    new_subcarrier_num_list.append((base_block_num_per_server + 1) * ue_num)

target_subcarrier_block_size = 0
real_subcarrier_core_num = 0
for b in range(160, 4, -4):
    cur_total_core_num = 0
    for i in range(num_server):
        cur_total_core_num += ceil_divide(new_subcarrier_num_list[i], b)
    if cur_total_core_num > target_core_num_for_subcarrier:
        target_subcarrier_block_size = b
        real_subcarrier_core_num = cur_total_core_num
        break
new_subcarrier_block_list = []
for i in range(num_server):
    new_subcarrier_block_list.append(target_subcarrier_block_size)
core_num_for_coding = core_left_for_proc - real_subcarrier_core_num

base_coding_core_per_server = core_num_for_coding / num_server
super_server_num = core_num_for_coding % num_server
new_coding_thread_num = []
for i in range(super_server_num):
    new_coding_thread_num.append(base_coding_core_per_server + 1)
for i in range(num_server - super_server_num):
    new_coding_thread_num.append(base_coding_core_per_server)

new_hydra_servers = []
for i in range(num_server):
    new_hydra_servers.append(app_node_list[i])

deploy_json['hydra_servers'] = new_hydra_servers
deploy_json['subcarrier_num_list'] = new_subcarrier_num_list
deploy_json['subcarrier_block_list'] = new_subcarrier_block_list
deploy_json['coding_thread_num'] = new_coding_thread_num

new_deploy_fn = deploy_fn[:-5] + "_{}c.json".format(target_core_num)
outf = open(new_deploy_fn, "w")
json.dump(deploy_json, outf)
outf.close()

new_template_fn = template_fn[:-5] + "_{}c.json".format(target_core_num)
outf = open(new_template_fn, "w")
json.dump(template_json, outf)
outf.close()