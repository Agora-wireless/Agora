"""
Resource Provisioner Class - prototype
"""
import socket
import sys
import struct
import time

# Network parameters
ip = "127.0.0.1"
rx_port = 4000 # Agora -> RP
tx_port = 3000 # Agora <- RP

def listen_data():
    data, address = s.recvfrom(4096)
    decoded_data = struct.unpack('NN', data) # 2 size_t type
    print("\n\n Server received: ", decoded_data, "\n\n")
    return decoded_data

def send_data(x, y):
    send_data = struct.pack('NN', int(x), int(y))
    s.sendto(send_data, (ip, tx_port))
    print("\n\n Server sent data: {}, {}\n\n".format(x, y))

if __name__ == "__main__":
    # Create a UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind the socket to the port
    server_address = (ip, rx_port)
    s.bind(server_address)
    print("Do Ctrl+c to exit the program !!")

    while True:

        data = listen_data()
        if data[0] + data[1] == 0:
            # traffic request message
            num = input("latency,curr#cores: ").split(',')
            send_data(num[0], num[1])
        else:
            # control message
            print("Add core: {}, remove core: {}".format(data[0], data[1]))

        # timer delay for 1 sec
        time.sleep(1)