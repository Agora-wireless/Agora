import csv
import time
import os.path
import matplotlib.pyplot as plt

def process_csv_file(file_path, last_processed_frame, prev_frame_number, prev_evm_reading):
    if not os.path.exists(file_path):
        return last_processed_frame, prev_frame_number, prev_evm_reading

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        lines = list(reader)

    if len(lines) <= last_processed_frame:
        return last_processed_frame, prev_frame_number, prev_evm_reading

    frame_numbers = []
    evm_readings = []

    for i in range(last_processed_frame, len(lines)):
        if len(lines[i]) < 2 or not lines[i][1].strip():
            continue

        try:
            frame_number = int(lines[i][0])
            evm_reading = float(lines[i][1])

            if (frame_number + 1) % 500 == 0:
                frame_numbers.append(frame_number)
                evm_readings.append(evm_reading)

                if prev_frame_number is not None and prev_evm_reading is not None:
                    frame_numbers.insert(0, prev_frame_number)
                    evm_readings.insert(0, prev_evm_reading)

                plt.plot(frame_numbers, evm_readings, 'b-')
                plt.xlabel('Frame Number')
                plt.ylabel('EVM Reading')
                plt.title('Real-time EVM Plot')
                plt.grid(True)
                plt.show(block=False)
                plt.pause(0.001)

                prev_frame_number = frame_number
                prev_evm_reading = evm_reading

                frame_numbers = []
                evm_readings = []

        except (ValueError, IndexError):
            continue

    return len(lines) - 1, prev_frame_number, prev_evm_reading

def monitor_csv_file(file_path):
    last_processed_frame = 0
    prev_frame_number = None
    prev_evm_reading = None

    while True:
        last_processed_frame, prev_frame_number, prev_evm_reading = process_csv_file(file_path, last_processed_frame, prev_frame_number, prev_evm_reading)
        time.sleep(1)

if __name__ == '__main__':
    csv_file_path = 'log-evm-BS.csv'  # Replace with the actual path to your CSV file

    while not os.path.exists(csv_file_path):
        time.sleep(1)

    monitor_csv_file(csv_file_path)
