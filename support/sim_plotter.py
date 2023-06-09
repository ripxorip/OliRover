import socket
import json
import os

# Shall source the environment variables instead

UDP_IP = "100.110.226.47"  # IP address the C program is sending to
UDP_PORT = 1447  # Port number the C program is sending to
BUFFER_SIZE = 1024

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    # Receive data from the socket
    data, addr = sock.recvfrom(BUFFER_SIZE)

    # Convert the received byte data to a string
    json_str = data.decode('utf-8')

    # Parse the JSON string into a Python dictionary
    json_data = json.loads(json_str)
    print(json_data )