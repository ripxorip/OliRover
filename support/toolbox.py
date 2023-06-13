from collections import deque
import socket
import json
import os
import sys
import time
import math
import typing
import evdev
from evdev import InputDevice, categorize, ecodes

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QGroupBox, QRadioButton, QSlider, QVBoxLayout, QHBoxLayout, QTextEdit, QLineEdit
from PyQt5.QtCore import QObject, pyqtSlot, QThread, pyqtSignal, Qt
import queue

class Settings:
    settings = {
        'kp' : {
            'min': 0.00,
            'max': 5.00,
        },
        'ki' : {
            'min': 0.00,
            'max': 5.00,
        },
        'kd' : {
            'min': 0.00,
            'max': 5.00,
        },
        'Placeholder 3' : {
            'min': 0,
            'max': 100,
        },
        'Placeholder 4' : {
            'min': 0,
            'max': 100,
        },
        'Placeholder 5' : {
            'min': 0,
            'max': 100,
        },
        'Placeholder 6' : {
            'min': 0,
            'max': 100,
        },
        'Placeholder 7' : {
            'min': 0,
            'max': 100,
        },
        'Placeholder 8' : {
            'min': 0,
            'max': 100,
        },
        'Placeholder 9' : {
            'min': 0,
            'max': 100,
        }
    }

class JoystickThread(QThread):
    def __init__(self, joystick_evdev, comm_queue):
        super().__init__()
        self.joystick_evdev = joystick_evdev
        self.comm_queue = comm_queue
        self.dev = InputDevice(self.joystick_evdev)

    def run(self):
        self.dev.grab()
        for event in self.dev.read_loop():
            ev = categorize(event)
            if ecodes.bytype[ev.event.type][ev.event.code] == 'ABS_RY':
                value = ev.event.value / 32768
                self.comm_queue.put({'type': 'joystick', 'axis': 'y', 'value': value})

            elif ecodes.bytype[ev.event.type][ev.event.code] == 'ABS_RX':
                value = ev.event.value / 32768
                self.comm_queue.put({'type': 'joystick', 'axis': 'x', 'value': value})

class CommThread(QThread):
    plot_data_signal = pyqtSignal(object)

    def __init__(self, comm_queue):
        super().__init__()
        self.comm_queu = comm_queue

        UDP_IP = os.environ['LOG_RECV_IP']
        UDP_PORT = int(os.environ['LOG_RECV_PORT'])
        self.buffer_size = 4096

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))

        self.CONTROLLER_SIM_COMMANDS_IP = os.environ['CONTROLLER_SIM_COMMANDS_IP']
        self.CONTROLLER_SIM_COMMANDS_PORT = int(os.environ['CONTROLLER_SIM_COMMANDS_PORT'])
        self.controller_sim_command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.has_received_settings = False

    def run(self):
        while(True):
            # Receive data from the socket
            data, addr = self.sock.recvfrom(self.buffer_size)

            # Convert the received byte data to a string
            json_str = data.decode('utf-8')

            # Parse the JSON string into a Python dictionary
            json_data = json.loads(json_str)
            if (json_data['type'] == 'parameters'):
                self.has_received_settings = True
            self.plot_data_signal.emit(json_data)

            if not self.has_received_settings:
                req = {'type': 'request_settings'}
                json_str = json.dumps(req)
                self.controller_sim_command_sock.sendto(json_str.encode('utf-8'), (self.CONTROLLER_SIM_COMMANDS_IP, self.CONTROLLER_SIM_COMMANDS_PORT))

            while not self.comm_queu.empty():
                json_data = self.comm_queu.get()
                json_str = json.dumps(json_data)
                self.controller_sim_command_sock.sendto(json_str.encode('utf-8'), (self.CONTROLLER_SIM_COMMANDS_IP, self.CONTROLLER_SIM_COMMANDS_PORT))

class ControlInterface(QWidget):
    def __init__(self, settings, comm_queue):
        super().__init__()
        self.plots = {}
        self.settings = settings
        self.sliders = {}
        self.text_inputs = {}
        self.comm_queue = comm_queue
        self.counter = 0
        grid = QGridLayout()

        grid.addWidget(self.createSlider('kp', settings['kp']), 0, 0)
        grid.addWidget(self.createSlider('ki', settings['ki']), 0, 1)
        grid.addWidget(self.createSlider('kd', settings['kd']), 0, 2)

        grid.addWidget(self.createSlider('Placeholder 3', settings['Placeholder 3']), 1, 0)
        grid.addWidget(self.createSlider('Placeholder 4', settings['Placeholder 4']), 1, 1)
        grid.addWidget(self.createSlider('Placeholder 5', settings['Placeholder 5']), 1, 2)
        grid.addWidget(self.createSlider('Placeholder 6', settings['Placeholder 6']), 2, 0)
        grid.addWidget(self.createSlider('Placeholder 7', settings['Placeholder 7']), 2, 1)
        grid.addWidget(self.createSlider('Placeholder 8', settings['Placeholder 8']), 2, 2)

         # Disable the sliders until each value has been received
        for key in self.sliders.keys():
            self.sliders[key].setEnabled(False)

        self.add_plot('PID Tuning', grid, 3)
        self.add_plot('PID Tuning 2', grid, 4)
        self.add_plot('PID Tuning 3', grid, 5)

        # Create circular buffer for plotting
        self.plot_data = {}
        self.plot_data['PID Tuning'] =  []
        self.plot_data['PID Tuning'].append(deque(maxlen=1000))

        self.plot_data['PID Tuning 2'] =  []
        self.plot_data['PID Tuning 2'].append(deque(maxlen=1000))

        self.plot_data['PID Tuning 3'] =  []
        self.plot_data['PID Tuning 3'].append(deque(maxlen=1000))

        self.setLayout(grid)

        self.setWindowTitle("Robot Configurator")
        self.resize(1600, 1200)
        self.show()

    def apply_plot_data(self):
        # Get the data from the queue
        for key in self.plots.keys():
            self.plots[key].plot(self.plot_data[key][0], clear=True)
            for i in range(1, len(self.plot_data[key])):
                self.plots[key].plot(self.plot_data[key][i], clear=False)

    def update_plots(self, data):
        if self.counter % 2 == 0:
            self.plot_data['PID Tuning'][0].append(data['sensor_angular_velocity_z'])
        if self.counter % 50 == 0:
            self.apply_plot_data()
            self.counter = 0
        self.counter += 1

    def createSlider(self, text, setting):
        groupBox = QGroupBox(text)

        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)

        range = setting['max'] - setting['min']
        slider.step = range / 100.00
        slider.setRange(0, 100)
        slider.setTickInterval(10)
        slider.setSingleStep(1)
        slider.setMinimum(0)
        slider.setMaximum(100)
        slider.setValue(0)
        slider.sliderMoved.connect(lambda: self.slider_value_changed(text))

        text_input = QLineEdit()
        text_input.setText(str(setting['current_value']))
        text_input.editingFinished.connect(lambda: self.line_edit_value_changed(text))

        hbox = QHBoxLayout()
        hbox.addWidget(slider)
        hbox.addWidget(text_input)
        hbox.setStretch(0, 4)  # Stretch the slider to twice the size
        hbox.setStretch(1, 1)  # Stretch the line edit to 1x size

        groupBox.setLayout(hbox)
        self.sliders[text] = slider
        self.text_inputs[text] = text_input
        return groupBox

    def add_plot(self, name, grid, position):
        # Add a graph to the GUI
        self.plots[name] = pg.PlotWidget()
        self.plots[name].setYRange(-5.00, 5.00)
        self.plots[name].setXRange(0, 1000)
        self.plots[name].showGrid(x=True, y=True)
        self.plots[name].setLabel('left', 'Y')
        self.plots[name].setLabel('bottom', 'X')
        self.plots[name].setTitle(name)
        grid.addWidget(self.plots[name], position, 0, 1, 3)

    def publish_setting(self, setting):
        self.comm_queue.put({'type': 'setting', 'data': {'setting': setting, 'value': self.settings[setting]['current_value']}})

    def slider_value_changed(self, setting):
        self.settings[setting]['current_value'] = self.sliders[setting].value() * self.sliders[setting].step
        num = self.sliders[setting].value() * self.sliders[setting].step
        num = round(num, 4)
        self.text_inputs[setting].setText(str(num))
        self.publish_setting(setting)

    def line_edit_value_changed(self, setting):
        val = self.settings[setting]['current_value'] = float(self.text_inputs[setting].text())
        self.sliders[setting].setValue(int(val / self.sliders[setting].step))
        self.publish_setting(setting)

    def connect_plot_data(self, recv_log_data_signal):
        recv_log_data_signal.plot_data_signal.connect(self.recv_log_data)

    @pyqtSlot(object)
    def recv_log_data(self, data):
        if data['type'] == 'log':
            self.update_plots(data['data'])
        elif data['type'] == 'parameters':
            for key in data['data']:
                self.sliders[key].setEnabled(True)
                self.settings[key]['current_value'] = data['data'][key]
                num = data['data'][key]
                num = round(num, 4)
                self.text_inputs[key].setText(str(num))
                self.sliders[key].setValue(int(num / self.sliders[key].step))
        return

def main():
    input_dev = None
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if 'Logitech Gamepad F310' == device.name:
            input_dev = device.path

    settings = Settings.settings
    for k in settings:
        s = settings[k]
        s['current_value'] = s['min']

    app = QApplication(sys.argv)
    q = queue.Queue()
    ci = ControlInterface(settings, q)
    ct = CommThread(q)
    jt = JoystickThread(input_dev, q)
    ci.connect_plot_data(ct)
    ct.start()
    jt.start()
    app.exec_()

if __name__ == '__main__':
    main()