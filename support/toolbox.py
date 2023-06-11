from collections import deque
import socket
import json
import os
import sys
import time
import math

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QGroupBox, QRadioButton, QSlider, QVBoxLayout, QHBoxLayout, QTextEdit, QLineEdit
from PyQt5.QtCore import pyqtSlot, QThread, pyqtSignal, Qt
import queue

class Settings:
    settings = {
        'Value 0' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 1' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 2' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 3' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 4' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 5' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 6' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 7' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 8' : {
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Value 9' : {
            'min': 0,
            'max': 100,
            'default': 50
        }
    }



class CommThread(QThread):
    plot_data_signal = pyqtSignal(object)

    def __init__(self, comm_queue):
        super().__init__()
        self.gui_queue = comm_queue

        UDP_IP = os.environ['LOG_RECV_IP']
        UDP_PORT = int(os.environ['LOG_RECV_PORT'])
        self.buffer_size = 4096

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))

        self.CONTROLLER_SIM_COMMANDS_IP = os.environ['CONTROLLER_SIM_COMMANDS_IP']
        self.CONTROLLER_SIM_COMMANDS_PORT = int(os.environ['CONTROLLER_SIM_COMMANDS_PORT'])
        self.controller_sim_command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        while(True):
            # Receive data from the socket
            data, addr = self.sock.recvfrom(self.buffer_size)

            # Convert the received byte data to a string
            json_str = data.decode('utf-8')

            # Parse the JSON string into a Python dictionary
            json_data = json.loads(json_str)
            self.plot_data_signal.emit(json_data)

            while not self.gui_queue.empty():
                json_data = self.gui_queue.get()
                json_str = json.dumps(json_data)
                self.controller_sim_command_sock.sendto(json_str.encode('utf-8'), (self.CONTROLLER_SIM_COMMANDS_IP, self.CONTROLLER_SIM_COMMANDS_PORT))

class PlotInterface(QWidget):
    def __init__(self, settings, comm_queue):
        super().__init__()
        self.plots = {}
        self.settings = settings
        self.sliders = {}
        self.gui_queue = comm_queue
        grid = QGridLayout()

        self.add_plot('PID Tuning', grid, 0)
        self.add_plot('PID Tuning 2', grid, 1)
        self.add_plot('PID Tuning 3', grid, 2)

        self.setLayout(grid)

        self.setWindowTitle("Robot Configurator")
        self.resize(1600, 1200)
        self.show()

    def add_plot(self, name, grid, position):
        # Add a graph to the GUI
        self.plots[name] = pg.PlotWidget()
        self.plots[name].setYRange(0, 4096)
        self.plots[name].setXRange(0, 100)
        self.plots[name].showGrid(x=True, y=True)
        self.plots[name].setLabel('left', 'Y')
        self.plots[name].setLabel('bottom', 'X')
        self.plots[name].setTitle(name)
        grid.addWidget(self.plots[name], position, 0, 1, 2)

class ControlInterface(QWidget):
    def __init__(self, settings, gui_queue):
        super().__init__()
        self.plots = {}
        self.settings = settings
        self.sliders = {}
        self.text_inputs = {}
        self.gui_queue = gui_queue
        self.counter = 0
        grid = QGridLayout()

        grid.addWidget(self.createSlider('Value 0', settings['Value 0']), 0, 0)
        grid.addWidget(self.createSlider('Value 1', settings['Value 1']), 1, 0)
        grid.addWidget(self.createSlider('Value 2', settings['Value 2']), 0, 1)
        grid.addWidget(self.createSlider('Value 3', settings['Value 3']), 1, 1)
        grid.addWidget(self.createSlider('Value 4', settings['Value 4']), 0, 2)
        grid.addWidget(self.createSlider('Value 5', settings['Value 5']), 1, 2)
        grid.addWidget(self.createSlider('Value 6', settings['Value 6']), 2, 0)
        grid.addWidget(self.createSlider('Value 7', settings['Value 7']), 2, 1)
        grid.addWidget(self.createSlider('Value 8', settings['Value 8']), 2, 2)

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
        # FIXME shall be float: https://stackoverflow.com/questions/20632841/qt-horizontalslider-send-float-values
        groupBox = QGroupBox(text)

        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(int((setting['max']-setting['min'])/10))
        slider.setSingleStep(1)
        slider.setMinimum(setting['min'])
        slider.setMaximum(setting['max'])
        slider.setValue(setting['current_value'])
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
        self.gui_queue.put({'setting': setting, 'value': self.settings[setting]['current_value']})

    def slider_value_changed(self, setting):
        self.settings[setting]['current_value'] = self.sliders[setting].value()
        self.text_inputs[setting].setText(str(self.sliders[setting].value()))
        self.publish_setting(setting)

    def line_edit_value_changed(self, setting):
        val = self.settings[setting]['current_value'] = float(self.text_inputs[setting].text())
        self.sliders[setting].setValue(val)
        self.publish_setting(setting)

    def connect_plot_data(self, recv_log_data_signal):
        recv_log_data_signal.plot_data_signal.connect(self.recv_log_data)

    @pyqtSlot(object)
    def recv_log_data(self, data):
        self.update_plots(data)
        return

def main():
    #FIXME The current settings shall come from the robot instead, sent using a custom message
    settings = Settings.settings
    for k in settings:
        s = settings[k]
        s['current_value'] = s['default']

    app = QApplication(sys.argv)
    q = queue.Queue()
    ci = ControlInterface(settings, q)
    ct = CommThread(q)
    ci.connect_plot_data(ct)
    ct.start()
    app.exec_()

if __name__ == '__main__':
    main()