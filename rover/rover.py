# -*- coding: utf-8 -*-
# vim:fenc=utf-8

import os
import sys
import inspect
import time

import typer
import subprocess

import socket

import threading
import queue

import json

from ctypes import *

# Main module thats intended to be run on the rover (Rpi)

# Import the C interface (For communication with the controller)
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, os.path.join(parentdir, 'interface'))
from interface import *

# The actual rover communicates with the outside world over JSON, (UDP)

class Rover:
    def __init__(self, communication_interface):
        self.cc = communication_interface

        api_rx_ip = os.environ['ROVER_API_RX_IP']
        api_rx_port = int(os.environ['ROVER_API_RX_PORT'])

        self.api_tx_ip = os.environ['ROVER_API_TX_IP']
        self.api_tx_port = int(os.environ['ROVER_API_TX_PORT'])

        self.api_rx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.api_rx_socket.bind((api_rx_ip, api_rx_port))

        self.api_tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.api_thread = threading.Thread(target=self.api_thread)
        self.api_thread.start()

        self.controller_thread = threading.Thread(target=self.controller_thread)
        self.controller_thread.start()

        self.cbwo = CircularBufferWithOffset(1024)

    # Get API data from e.g. the toolbox
    def handle_api_call(self, data):
        # FIXME Cont. here...
        # handle parameters etc
        print(data)

    def api_thread(self):
        while True:
            data, addr = self.api_rx_socket.recvfrom(4096)
            res = self.handle_api_call(data)
            # self.api_tx_socket.sendto(data, (self.api_tx_ip, self.api_tx_port))

    def controller_data_send(self, id, data):
        # Get the data as a bytes object from the ctypes struct
        data_bytes = bytes(data)
        bytes_to_send = id.to_bytes(4, byteorder='little') + data_bytes + (id ^ 0xffffffff).to_bytes(4, byteorder='little')
        self.cc.write(bytes_to_send)

    def verify_controller_message(self, id, message_len):
        message_data = [0] * 1024
        id_flip = id ^ 0xffffffff
        # Read the first 4 bytes from the cbwo
        read_id_end = 0
        for i in range(0, 4):
            _byte = self.cbwo.get_value(i)
            read_id_end |= _byte << ((3-i) * 8)

        read_id_start = 0
        start_offset = sizeof(c_uint32) + message_len
        for i in range(start_offset, start_offset + sizeof(c_uint32)):
            _byte = self.cbwo.get_value(i)
            read_id_start |= _byte << ((3-(i-start_offset)) * 8)

        if read_id_start == id and read_id_end == id_flip:
            total_len = message_len + 2 * sizeof(c_uint32)
            for i in range(0, total_len):
                message_data[i] = self.cbwo.get_value(i)
            message_data = message_data[0:total_len]
            # Reverse the message_data
            message_data = message_data[::-1]
            # Remove the first and last 4 bytes
            message_data = message_data[4:-4]
            return bytes(message_data)

        return None

    def controller_data_received(self, data):
        for d in data:
            self.cbwo.write(d)
            message_bytes = self.verify_controller_message(INTERFACE_SENSORS, sizeof(interface_sensors_t()))
            if message_bytes is not None:
               res_struct = interface_sensors_t()
               memmove(pointer(res_struct), message_bytes, sizeof(res_struct))
               sensor_data = getdict(res_struct)

               api_sensor_data = {'type': 'log', 'data': sensor_data}
               # Send the sensor data to the Toolbox for logging
               api_sensor_data_json = json.dumps(api_sensor_data)
               self.api_tx_socket.sendto(api_sensor_data_json.encode('utf-8'), (self.api_tx_ip, self.api_tx_port))

               echo_data = interface_sensors_t()
               echo_data.angular_velocity_x = sensor_data['angular_velocity_x']
               echo_data.angular_velocity_y = sensor_data['angular_velocity_y']
               echo_data.angular_velocity_z = sensor_data['angular_velocity_z']

               echo_data.linear_acceleration_x = sensor_data['linear_acceleration_x']
               echo_data.linear_acceleration_y = sensor_data['linear_acceleration_y']
               echo_data.linear_acceleration_z = sensor_data['linear_acceleration_z']

               self.controller_data_send(INTERFACE_SENSORS, echo_data)

    def controller_thread(self):
        while True:
            while not self.cc.data_queue.empty():
                data = self.cc.data_queue.get()
                self.controller_data_received(data)
            time.sleep(0.01)

class CircularBufferWithOffset:
    def __init__(self, size):
        self.size = size
        self.buffer = [0] * size
        self.start = 0
        self.end = self.size - 1

    def increment_pointers(self):
        self.start = (self.start + 1) % self.size
        self.end = (self.end + 1) % self.size

    def write(self, data):
        self.buffer[self.start] = data
        self.increment_pointers()

    def get_value(self, offset):
        index = self.end - offset
        if index < 0:
            index = self.size + index
        return self.buffer[index]

# These classes shall be broken out into separate files
class ControllerInterfaceUDP:
    def __init__(self, data_queue):
        # Start the controller using subprocess
        # FIXME Make sure the controller is built first in tasks.json
        self.controller = subprocess.Popen(["./controller/build/OliRoverControllerSimulator"])
        self.read_thread_handle = threading.Thread(target=self.read_thread)
        self.read_thread_handle.start()
        self.data_queue = data_queue

    def read_thread(self):
        # Use a queue to read data from the controller
        # this function returns the oldest data in the queue
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.controller_tx_ip = os.environ['CONTROLLER_TX_IP']
        self.controller_tx_port = int(os.environ['CONTROLLER_TX_PORT'])

        self.controller_rx_ip = os.environ['CONTROLLER_RX_IP']
        self.controller_rx_port = int(os.environ['CONTROLLER_RX_PORT'])

        self.controller_rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sock.bind((self.controller_tx_ip, self.controller_tx_port))

        while True:
            data, addr = self.sock.recvfrom(4096)
            self.data_queue.put(data)

    def write(self, data):
        self.controller_rx_sock.sendto(data, (self.controller_rx_ip, self.controller_rx_port))

class ControllerInterfaceSerial:
    def __init__(self):
        pass

def getdict(struct):
    return dict((field, getattr(struct, field)) for field, _ in struct._fields_)

# The interface towards this module is always UDP with JSON independet of simulation or not

# Main function
def main(
        simulation: bool = typer.Option(False, help="Run in simulation mode"),
    ):

    q = queue.Queue()
    if simulation:
        cc = ControllerInterfaceUDP(q)
    else:
        print("Running on target")
        cc = ControllerInterfaceSerial(q)

    rover = Rover(cc)

    # FIXME refactor interface_ to controller_interface
    inp = interface_input_t()
    while True:
        time.sleep(1)

if __name__ == "__main__":
    typer.run(main)