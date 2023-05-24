import csv
import time
import os.path

def process_csv_file(file_path, last_processed_row):
    if not os.path.exists(file_path):
        return last_processed_row

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        lines = list(reader)

    if len(lines) <= last_processed_row:
        return last_processed_row

    for i in range(last_processed_row, len(lines)):
        if len(lines[i]) < 2 or not lines[i][1].strip():
            continue

        frame_number = int(lines[i][0])
        evm_reading = float(lines[i][1])

        if (frame_number + 1) % 500 == 0:
            print(f"Frame Number: {frame_number}, EVM Reading: {evm_reading}")

    return len(lines) - 1

def monitor_csv_file(file_path):
    last_processed_row = 0

    while True:
        last_processed_row = process_csv_file(file_path, last_processed_row)
        time.sleep(1)

if __name__ == '__main__':
    csv_file_path = 'log-evm-BS.csv'  # Replace with the actual path to your CSV file

    while not os.path.exists(csv_file_path):
        time.sleep(1)

    monitor_csv_file(csv_file_path)
