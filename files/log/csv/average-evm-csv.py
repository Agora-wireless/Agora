import csv

def calculate_average_evm(csv_file):
    evm_sum = 0
    evm_count = 0

    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header row

        # Find the starting frame
        start_frame = 1500

        # Iterate over each row in the CSV file
        for row in reader:
            frame = int(row[0])
            evm = float(row[1])

            # Check if frame is within the desired range
            if frame >= start_frame:
                evm_sum += evm
                evm_count += 1

    # Calculate the average EVM
    average_evm = evm_sum / evm_count if evm_count > 0 else 0

    return average_evm

# Usage example
csv_file_path = 'log-evm-BS.csv'  # Replace with your CSV file path
average_evm = calculate_average_evm(csv_file_path)
print(f"Average EVM: {average_evm}")

