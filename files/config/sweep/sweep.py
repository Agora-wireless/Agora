import json

settings = {
    "fft_sizes" : [ 11, 12, 13 ],
    "ofdm_data_nums" : [ 21, 22, 23 ]
}

def create_config_file( index, fft_size, ofdm_data_num ):
  with open(f'./config_outputs/config_{index}.json', "w") as write_file:
    json.dump(settings, write_file)

current_index = 0

for fft_size in settings["fft_sizes"]:
  current_ofdm_data_num = 0
  current_index = current_index + 1
  for ofdm_data_num in settings["ofdm_data_nums"]:
    current_ofdm_data_num = current_ofdm_data_num + 1
    create_config_file( current_index, fft_size, current_ofdm_data_num )