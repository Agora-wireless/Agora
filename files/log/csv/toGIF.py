import csv
import time
import os.path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def process_csv_file(file_path, last_processed_frame, prev_frame_number, prev_reading, ax, ylabel, line_color):
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

                prev_frame_number = frame_number
                prev_reading = reading

                frame_numbers = []
                readings = []

        except (ValueError, IndexError):
            continue

    return len(lines) - 1, prev_frame_number, prev_reading

def monitor_csv_files(evm_file_path, snr_file_path):
    last_processed_frame_evm = 0
    prev_frame_number_evm = None
    prev_evm_reading = None

    last_processed_frame_snr = 0
    prev_frame_number_snr = None
    prev_snr_reading = None

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    fig.suptitle('Agora mmWave Performance')

    def animate(i):
        nonlocal last_processed_frame_evm, prev_frame_number_evm, prev_evm_reading
        nonlocal last_processed_frame_snr, prev_frame_number_snr, prev_snr_reading

        last_processed_frame_evm, prev_frame_number_evm, prev_evm_reading = process_csv_file(
            evm_file_path, last_processed_frame_evm, prev_frame_number_evm, prev_evm_reading, ax1, 'EVM (%)', 'blue')
        last_processed_frame_snr, prev_frame_number_snr, prev_snr_reading = process_csv_file(
            snr_file_path, last_processed_frame_snr, prev_frame_number_snr, prev_snr_reading, ax2, 'SNR (db)', 'red')

    anim = FuncAnimation(fig, animate, interval=10)

    plt.tight_layout()
    plt.show()

    # Save the animation as a GIF
    anim.save('realtime_plot.gif', writer='pillow')

if __name__ == '__main__':
    evm_csv_file_path = 'log-evm-BS.csv'  # Replace with the actual path to your EVM CSV file
    snr_csv_file_path = 'log-ulsnr-BS.csv'  # Replace with the actual path to your SNR CSV file

    while not os.path.exists(evm_csv_file_path) or not os.path.exists(snr_csv_file_path):
        time.sleep(1)

    monitor_csv_files(evm_csv_file_path, snr_csv_file_path)
