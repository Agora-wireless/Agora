import csv
import time
import os.path
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os

def process_csv_file(file_path, last_processed_frame, prev_frame_number, prev_reading, ax, ylabel, line_color, line_color2, log_scale=False, legend_added=False):
    if not os.path.exists(file_path):
        return last_processed_frame, prev_frame_number, prev_reading, legend_added

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        lines = list(reader)

    if len(lines) <= last_processed_frame:
        return last_processed_frame, prev_frame_number, prev_reading, legend_added

    frame_numbers = []
    readings1 = []
    readings2 = []

    for i in range(last_processed_frame, len(lines)):
        if len(lines[i]) < 3 or not lines[i][1].strip() or not lines[i][2].strip():
            continue

        try:
            frame_number = int(lines[i][0])
            reading1 = float(lines[i][1])
            reading2 = float(lines[i][2])

            if (ylabel == 'BER') and (reading1 == 0):
                reading1 = float(0.0001)
            if (ylabel == 'BER') and (reading2 == 0):
                reading2 = float(0.0001)

            if (frame_number + 1) % 500 == 0:
                frame_numbers.append(frame_number)
                readings1.append(reading1)
                readings2.append(reading2)

                if prev_frame_number is not None and prev_reading is not None:
                    frame_numbers.insert(0, prev_frame_number)
                    readings1.insert(0, prev_reading[0])
                    readings2.insert(0, prev_reading[1])

                ax.plot(frame_numbers, readings1, color=line_color, linewidth=5.5, marker='o', markersize=10, label="User 0")
                ax.plot(frame_numbers, readings2, color=line_color2, linewidth=5.5, marker='o', markersize=10, label="User 1")

                if not legend_added:
                    ax.legend()  # Add legend if it has not been added yet
                    legend_added = True

                ax.set_xlabel('Frame Number')
                ax.set_ylabel(ylabel)

                if ylabel == 'EVM (%)':
                    ax.set_ylim([0, 40])
                if ylabel == 'SNR (dB)':
                    ax.set_ylim([0, 40])

                if ylabel == 'BER':
                    ax.set_yscale('log')
                    ax.set_ylim([1e-5, 1])

                    # Custom y-tick values
                    ax.set_yticks([1e-4, 1e-3, 1e-2, 1e-1, 1])
                    ax.set_yticklabels(['1e-4', '1e-3', '1e-2', '1e-1', '1'])

                prev_frame_number = frame_number
                prev_reading = (reading1, reading2)

                frame_numbers = []
                readings1 = []
                readings2 = []

        except (ValueError, IndexError):
            continue

    return len(lines) - 1, prev_frame_number, prev_reading, legend_added


def calculate_throughput(data_rate, ber):
    return data_rate * (1 - ber)

def monitor_csv_files(evm_file_path, snr_file_path, ber_file_path, data_rate):
    last_processed_frame_evm = 0
    prev_frame_number_evm = None
    prev_evm_reading = None

    last_processed_frame_snr = 0
    prev_frame_number_snr = None
    prev_snr_reading = None

    last_processed_frame_ber = 0
    prev_frame_number_ber = None
    prev_ber_reading = None

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
    fig.suptitle('Agora mmWave Performance')

    plt.ion()  # Enable interactive mode

    legend_added_evm = False;
    legend_added_snr = False;
    legend_added_ber = False;

    while True:
        ax1.set_xlabel('Frame Number', fontsize=18)
        ax1.set_ylabel('EVM (%)', fontsize=18)
        ax1.tick_params(axis='both', which='major', labelsize=12)

        ax2.set_xlabel('Frame Number', fontsize=18)
        ax2.set_ylabel('SNR (dB)', fontsize=18)
        ax2.tick_params(axis='both', which='major', labelsize=12)

        ax3.set_xlabel('Frame Number', fontsize=18)
        ax3.set_ylabel('BER', fontsize=18)
        ax3.tick_params(axis='both', which='major', labelsize=12)
        

        last_processed_frame_evm, prev_frame_number_evm, prev_evm_reading, legend_added_evm = process_csv_file(evm_file_path, last_processed_frame_evm, prev_frame_number_evm, prev_evm_reading, ax1, 'EVM (%)', 'blue', '#1f77b4', legend_added=legend_added_evm)
        last_processed_frame_snr, prev_frame_number_snr, prev_snr_reading, legend_added_snr = process_csv_file(snr_file_path, last_processed_frame_snr, prev_frame_number_snr, prev_snr_reading, ax2, 'SNR (dB)', 'red', '#d62728', legend_added=legend_added_snr)
        last_processed_frame_ber, prev_frame_number_ber, prev_ber_reading, legend_added_ber = process_csv_file(ber_file_path, last_processed_frame_ber, prev_frame_number_ber, prev_ber_reading, ax3, 'BER', 'green', '#2ca02c', legend_added=legend_added_ber)

        plt.tight_layout()
        plt.pause(0.001)

        time.sleep(0.1)


if __name__ == '__main__':
    cmd1 = "rm log-evm-BS.csv"
    cmd2 = "rm log-ulsnr-BS.csv"
    cmd3 = "rm log-ber-BS.csv"
    os.system(cmd1)
    os.system(cmd2)
    os.system(cmd3)

    evm_csv_file_path = 'log-evm-BS.csv'  # Replace with the actual path to your EVM CSV file
    snr_csv_file_path = 'log-ulsnr-BS.csv'  # Replace with the actual path to your SNR CSV file
    ber_csv_file_path = 'log-ber-BS.csv'  # Replace with the actual path to your BER CSV file
    data_rate = 50000  # Replace with the actual data rate in Mbps

    while not os.path.exists(evm_csv_file_path) or not os.path.exists(snr_csv_file_path) or not os.path.exists(ber_csv_file_path):
        time.sleep(0.1)

    monitor_csv_files(evm_csv_file_path, snr_csv_file_path, ber_csv_file_path, data_rate)
