import csv
import time
import os.path
import matplotlib.pyplot as plt
import numpy as np
import os

def process_csv_file(file_path, last_processed_frame, prev_frame_number, prev_reading, ax, ylabel, line_color, log_scale=False):
    if not os.path.exists(file_path):
        return last_processed_frame, prev_frame_number, prev_reading

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        lines = list(reader)

    if len(lines) <= last_processed_frame:
        return last_processed_frame, prev_frame_number, prev_reading

    frame_numbers = []
    readings = []

    for i in range(last_processed_frame, len(lines)):
        if len(lines[i]) < 2 or not lines[i][1].strip():
            continue

        try:
            frame_number = int(lines[i][0])
            reading = float(lines[i][1])

            if (frame_number + 1) % 500 == 0:
                frame_numbers.append(frame_number)
                readings.append(reading)

                if prev_frame_number is not None and prev_reading is not None:
                    frame_numbers.insert(0, prev_frame_number)
                    readings.insert(0, prev_reading)

                ax.plot(frame_numbers, readings, color=line_color)
                ax.set_xlabel('Frame Number')
                ax.set_ylabel(ylabel)

                if log_scale:
                    readings = np.where(np.array(readings) == 0, 1e-6, readings)  # Treat 0 as a very small value
                    ax.set_yscale('log')
                    ax.set_ylim([np.min(readings), np.max(readings)])  # Adjust y-axis limits based on data

                prev_frame_number = frame_number
                prev_reading = reading

                frame_numbers = []
                readings = []

        except (ValueError, IndexError):
            continue

    return len(lines) - 1, prev_frame_number, prev_reading

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

    while True:
        last_processed_frame_evm, prev_frame_number_evm, prev_evm_reading = process_csv_file(evm_file_path, last_processed_frame_evm, prev_frame_number_evm, prev_evm_reading, ax1, 'EVM (%)', 'blue')
        last_processed_frame_snr, prev_frame_number_snr, prev_snr_reading = process_csv_file(snr_file_path, last_processed_frame_snr, prev_frame_number_snr, prev_snr_reading, ax2, 'SNR (dB)', 'red')
        last_processed_frame_ber, prev_frame_number_ber, prev_ber_reading = process_csv_file(ber_file_path, last_processed_frame_ber, prev_frame_number_ber, prev_ber_reading, ax3, 'BER)', 'green')

        plt.tight_layout()
        plt.pause(0.001)

        time.sleep(1)

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
    data_rate = 3371 # Replace with the actual data rate in Mbps

    while not os.path.exists(evm_csv_file_path) or not os.path.exists(snr_csv_file_path) or not os.path.exists(ber_csv_file_path):
        time.sleep(1)

    monitor_csv_files(evm_csv_file_path, snr_csv_file_path, ber_csv_file_path, data_rate)
