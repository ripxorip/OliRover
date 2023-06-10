import serial
import math
import time
import os
import sys
import inspect
import zlib
import glob
from elftools.elf.elffile import ELFFile
from ctypes import c_int16, c_int32, memmove, pointer, sizeof, Structure
from struct import *
from rich.progress import Progress
import pickle

from art import *
import typer
from rich.console import Console
console = Console()

from pynput.keyboard import Key, Controller
from pathlib import Path
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QGroupBox, QRadioButton, QSlider, QVBoxLayout, QTextEdit
from PyQt5.QtCore import pyqtSlot, QThread, pyqtSignal, Qt

import queue
from enum import Enum

from ctypes import *

# Generate the C interface
sim = sys.argv[1] == 'simulate'
if sim:
    os.system("cd interface && clang2py --clang-args='-DSENSOR_DEBUG_LOG=1' frame.h > frame.py")
else:
    os.system("cd interface && clang2py frame.h > frame.py")
# Import the C interface
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, os.path.join(parentdir, 'interface'))
from frame import *

##
# Code begin
#

def getdict(struct):
    return dict((field, getattr(struct, field)) for field, _ in struct._fields_)

def await_reset(device):
    portFoundOrphan = False
    portFoundAgain = False
    # Instead of busy wait
    for i in range(0, 300):
        ports = glob.glob('/dev/ttyACM*')
        if not portFoundOrphan:
            if not device in ports:
                portFoundOrphan = True
        else:
            if device in ports:
                portFoundAgain = True
                break
        time.sleep(0.01)

    return portFoundAgain

class CBWO:
    def __init__(self, size):
        self.start = 0
        self.size = size
        self.buffer = [0] * self.size
        self.end = self.size - 1

    def increment_pointers(self):
        self.start = (self.start+1) % self.size
        self.end = (self.end+1) % self.size

    def write(self, value):
        self.buffer[self.start] = value
        self.increment_pointers()

    def get_value(self, offset):
        index = self.end - offset;
        if index < 0:
            index += self.size;
        return self.buffer[index]

class Settings:
    settings = {
        'Debounce time' : {
            'id': SETTING_DEBOUNCE_TIME,
            'min': 0,
            'max': 100,
            'default': 50
        },
        'Y Threshold' : {
            'id': SETTING_Y_THRESH,
            'max': 4096,
            'min': 0,
            'default': int(4096 * 0.75)
        },
        'X Threshold' : {
            'id': SETTING_X_THRESH,
            'max': 100,
            'min': 0,
            'default': 20
        }
    }


class Keymap:
    keymap_base = [
        # Left Pinky
        ###############################
                    'q',
        'z',        None,     'x',
                    'a',

        # Left ring
        ###############################
                    'w',
        'c',       None,     'd',
                    'r',

        # Left Middle
        ###############################
                    'f',
        None,       None,     'b',
                    's',

        # Left Index
        ###############################
                    'p',
        'v',        None,     'g',
                    't',

        # Left Thumb
        ###############################
                     ' ',
        ' ',  None,     Key.esc,
                    None,

        # Right thumb
        ###############################
                Key.backspace,
Key.backspace,      None,     '\n',
                    None,

        # Right index
        ###############################
                    'l',
        'm',        None,     'k',
                    'n',

        # Right middle
        ###############################
                    'u',
        'j',        None,     'h',
                    'e',

        # Right Ring
        ###############################
                    'y',
        ',',        None,     '.',
                    'i',

        # Right pinky
        ###############################
                    ';',
        None,       None,     '/',
                    'o'
    ]

class MagslideSIL:
    def __init__(self, data_dir=None, analyze_sensor=''):
        self.analyze_sensor=analyze_sensor
        if self.analyze_sensor != '':
            self.sensor_debug_slots = 4
        self.in_the_loop = False
        if data_dir != None:
            self.data_dir = data_dir
            self.load_data()
        else:
            self.keyboard = Controller()
            self.in_the_loop = True
        self.tester = 4
        self.matrix_n_1 = [0] * 50
        self.matrix = [0] * 50
        self.palette = ['#F0D9B1', '#E76258', '#898E8B', '#684853', '#748FC2']

    def setup(self, settings):
        self.so = CDLL('node/x86_sil/build/libnode_sil.so')
        self.nodes = []
        for i in range(10):
            self.nodes.append(node_t())
            self.so.node_init(POINTER(node_t)(self.nodes[i]))
        for k in settings:
            s = settings[k]
            ret = self.so.settings_set(s['id'], s['current_value'])
            if (ret != 1):
                print("Failed to set SIL setting")

    def load_data(self):
        self.raw_data = []
        # After this we use indexing from 0
        for i in range(1, 10+1):
            data = pickle.load(open('{}/node_{}.data'.format(self.data_dir, i), 'rb'))
            self.raw_data.append(data)

    # Main function of the simulator (handles the i2c reports from each node)
    def run_controller_sil(self, node_reports):
        # Called once every sample
        # Run the matrix logic
        for i in range(len(node_reports)):
            # Highlight the activations
            matrix_offset = i * 5
            s = node_reports[i]
            self.matrix[matrix_offset] = s.up
            self.matrix[matrix_offset+1] = s.left
            self.matrix[matrix_offset+2] = 0 # Left for Z
            self.matrix[matrix_offset+3] = s.right
            self.matrix[matrix_offset+4] = s.down

        for i in range(len(self.matrix)):
            if self.matrix[i] == 1 and self.matrix_n_1[i] == 0 and Keymap.keymap_base[i] != None:
                if self.in_the_loop:
                    self.keyboard.press(Keymap.keymap_base[i])
                else:
                    print(Keymap.keymap_base[i], end='')
            elif self.matrix[i] == 0 and self.matrix_n_1[i] == 1 and Keymap.keymap_base[i] != None:
                if self.in_the_loop:
                    self.keyboard.release(Keymap.keymap_base[i])

        self.matrix_n_1 = self.matrix[:]

        # Highlight the activations (only for debug/visualization)
        if not self.in_the_loop:
            for i in range(len(node_reports)):
                self.activations[i].append(0)
                r = node_reports[i]
                if r.up or r.down or r.left or r.right:
                    self.activations[i][len(self.activations[i])-1] = 4096

    def process(self):
        num_samples = len(self.raw_data[0]['up'])
        self.activations = []
        for i in range(10): self.activations.append([])

        if self.analyze_sensor != '':
            self.sensor_debug = []
            for i in range(10):
                self.sensor_debug.append([])
                for j in range(4):
                    self.sensor_debug[i].append([])
                    for k in range(self.sensor_debug_slots):
                        self.sensor_debug[i][j].append([])

        for j in range(num_samples):
            responses = []
            for i in range(10):
                sample = sensor_data_t()
                # Shall be raw data after logging new values from the new platform.c
                sample.values.up = int(self.raw_data[i]['up'][j])
                sample.values.left = int(self.raw_data[i]['left'][j])
                sample.values.right = int(self.raw_data[i]['right'][j])
                sample.values.down = int(self.raw_data[i]['down'][j])

                sample_pointer = POINTER(sensor_data_t)(sample)
                self.so.node_process(sample_pointer, POINTER(node_t)(self.nodes[i]))

                report = node_report_t()
                report_pointer = POINTER(node_report_t)(report)
                self.so.node_get_report(report_pointer, POINTER(node_t)(self.nodes[i]))

                if self.analyze_sensor != '':
                    # Fetch sensor results
                    # Each sensor
                    for k in range(4):
                        # Debug messages
                        for l in range(self.sensor_debug_slots):
                            self.sensor_debug[i][k][l].append(self.nodes[i].sensors[k].debug_log[l])

                responses.append(report)
            self.run_controller_sil(responses)
        print('')

    def run(self):
        # Run the simulation
        self.process()
        if self.analyze_sensor == '':
            # Display the standard results
            self.present_results()
        else:
            nsp = self.analyze_sensor.split(':')
            node = int(nsp[0])
            sensor = int(nsp[1])
            if sensor == 4:
                # Plot all combined
                self.present_sensor_analysis_combined(node)
            else:
                self.present_sensor_analysis(node, sensor)

    def present_sensor_analysis_combined(self, node):
        title = "Node {} Sensor combined analysis".format(node)
        app = pg.mkQApp("Magslide SIL results")
        anal_win = pg.GraphicsLayoutWidget(show=True, title=title)
        anal_win.resize(2600,1350)
        anal_win.setWindowTitle(title)
        pp = anal_win.addPlot(title=title)
        keys = ['right', 'up', 'left', 'down']

        for i in range(4):
            pp.plot(self.raw_data[node-1][keys[i]], pen=pg.mkPen(color=self.palette[i], width=2))

        for i in range(self.sensor_debug_slots):
            anal_win.nextRow()
            p = anal_win.addPlot(title='Slot {}'.format(i))
            for j in range(4):
                p.plot(self.sensor_debug[node-1][j][i], pen=pg.mkPen(color=self.palette[j], width=2))
            p.setXLink(pp)

        pg.setConfigOptions(antialias=True)
        pg.exec()

    def present_sensor_analysis(self, node, sensor):
        title = "Node {} Sensor {} analysis".format(node, sensor)
        app = pg.mkQApp("Magslide SIL results")
        anal_win = pg.GraphicsLayoutWidget(show=True, title=title)
        anal_win.resize(2600,1350)
        anal_win.setWindowTitle(title)
        pp = anal_win.addPlot(title=title)
        keys = ['right', 'up', 'left', 'down']
        pp.plot(self.raw_data[node-1][keys[sensor]], pen=pg.mkPen(color=self.palette[1], width=2))

        for i in range(self.sensor_debug_slots):
            anal_win.nextRow()
            p = anal_win.addPlot(title='Slot {}'.format(i))
            p.plot(self.sensor_debug[node-1][sensor][i], pen=pg.mkPen(color=self.palette[2], width=2))
            p.setXLink(pp)

        pg.setConfigOptions(antialias=True)
        pg.exec()

    def plot_node_data(self, win, node_data, yrange=None):
        for i in range(10):
            p = win.addPlot(title="Node {}".format(i+1))
            p.plot(node_data[i]['up'], pen=pg.mkPen(color=self.palette[0], width=2))
            p.plot(node_data[i]['left'], pen=pg.mkPen(color=self.palette[1], width=2))
            p.plot(node_data[i]['right'], pen=pg.mkPen(color=self.palette[2], width=2))
            p.plot(node_data[i]['down'], pen=pg.mkPen(color=self.palette[3], width=2))
            p.plot(self.activations[i], pen=pg.mkPen(color=self.palette[4], width=2))
            if yrange != None:
                p.setYRange(yrange[0], yrange[1])
            if (i == 4):
                win.nextRow()

    def present_results(self):
        app = pg.mkQApp("Magslide SIL results")

        plot_raw_win = True
        plot_norm_win = False
        plot_analysis_win = False

        if plot_raw_win:
            raw_win = pg.GraphicsLayoutWidget(show=True, title="Magslide raw data")
            raw_win.resize(2600,1350)
            raw_win.setWindowTitle('Magslide raw data')
            self.plot_node_data(raw_win, self.raw_data)

        if plot_norm_win:
            norm_win = pg.GraphicsLayoutWidget(show=True, title="Magslide normalized data")
            norm_win.resize(2600,1350)
            norm_win.setWindowTitle('Magslide normalized data')
            self.plot_node_data(norm_win, self.normalized_data, [0, 4096])

        pg.setConfigOptions(antialias=True)
        pg.exec()


class Pollthread(QThread):
    magic_signal = pyqtSignal(object)

    def __init__(self, controller, gui_queue):
        super().__init__()
        self.controller = controller
        self.gui_queue = gui_queue
        self.shall_run = True

    def run(self):
        sil = MagslideSIL()
        while(self.shall_run):
            # Check the queue for GUI updates
            while not self.gui_queue.empty():
                setting = self.gui_queue.get()
                self.controller.settings_set_all_nodes(setting['id'], setting['current_value'])

            node_reports = []
            reports = self.controller.get_all_nodes_samples().reports
            for i in range(10):
                node_reports.append(reports[i])
            sil.run_controller_sil(node_reports)

            # Look for Magic pattern */
            shall_record = True
            for i in range(4):
                if node_reports[i].up != 1:
                    shall_record = False
            if shall_record:
                self.magic_signal.emit('shall_sil')

    def stop(self):
        self.shall_run = False

class MagslideGUI(QWidget):
    def __init__(self, settings, gui_queue):
        super().__init__()
        self.settings = settings
        self.sliders = {}
        self.gui_queue = gui_queue
        grid = QGridLayout()
        grid.addWidget(self.createExampleGroup('Y Threshold', settings['Y Threshold']), 0, 0)
        grid.addWidget(self.createExampleGroup('X Threshold', settings['X Threshold']), 1, 0)
        grid.addWidget(self.createExampleGroup('Debounce time', settings['Debounce time']), 0, 1)
        self.te = QTextEdit(self)
        grid.addWidget(self.te, 2, 0, 4, 2)
        # grid.addWidget(self.createExampleGroup('Not used', settings), 1, 1)

        # Add text input widget for test typing? :)
        self.setLayout(grid)

        self.setWindowTitle("Magslide Configurator")
        self.resize(1200, 200)
        self.show()

    def createExampleGroup(self, text, setting):
        groupBox = QGroupBox(text)

        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(int((setting['max']-setting['min'])/10))
        slider.setSingleStep(1)
        slider.setMinimum(setting['min'])
        slider.setMaximum(setting['max'])
        slider.setValue(setting['current_value'])
        slider.sliderReleased.connect(lambda: self.slider_value_changed(text))

        vbox = QVBoxLayout()
        vbox.addWidget(slider)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        self.sliders[text] = slider
        return groupBox

    def slider_value_changed(self, setting):
        self.settings[setting]['current_value'] = self.sliders[setting].value()
        self.gui_queue.put(self.settings[setting])
        print('[{}] changed, value: {}'.format(setting, self.sliders[setting].value()))

    def connect_magic(self, magic_signal):
        magic_signal.magic_signal.connect(self.magic_slot)

    @pyqtSlot(object)
    def magic_slot(self, data):
        self.shall_sil = True
        self.close()
        return

class MagslideController:
    def __init__(self, port):
        self.rcv_in_progress = False
        self.rcv_data = [0] * 64
        self.rcd_data_ptr = 0
        self.cb = CBWO(8)
        self.port = port

    def connect(self):
        self.ser = serial.Serial(self.port, 115200)

    def read_msg(self):
        # FIXME Use timeout for the serial port in a clever way
        while True:
            cc = self.ser.read()
            for c in cc:
                ret = self.handle_byte(c)
                if ret != None:
                    # We got our frame
                    return(ret)

    def handle_byte(self, byte_in):
        self.cb.write(byte_in)
        # Return a parsed frame if all is parsed
        if not self.rcv_in_progress:
            magic = []
            for i in range(0, 4):
                magic.append(self.cb.get_value(3-i))
            # Check Magic
            magic_int = int.from_bytes(bytes(magic), byteorder='little')
            if magic_int == 0xB16B00B5:
                magic = bytes(magic)
                self.rcv_in_progress = True
                self.rcv_data_ptr = 4
                for i in range(0, 4):
                    self.rcv_data[i] = magic[i]
        else:
            self.rcv_data[self.rcv_data_ptr] = byte_in
            self.rcv_data_ptr += 1
            if self.rcv_data_ptr > 11:
                frlen = self.rcv_data[11]
                if frlen <= 52 and self.rcv_data_ptr > frlen + 11:
                    # The message is now reveived
                    self.rcv_in_progress = False
                    return self.parse_frame(bytes(self.rcv_data))
        return None

    def parse_frame(self, bytes_in):
        # FIXME shall check CRC etc..
        res_struct = ext_command_t()
        memmove(pointer(res_struct), bytes_in, sizeof(res_struct))
        # Create data using the length of the res_struct
        dd = []
        for i in range(0, res_struct.len):
            dd.append(res_struct.data[i])
        ddict = getdict(res_struct)
        ddict['data'] = dd
        return ddict

    def create_frame(self, node, shall_respond, frame_data):
        ff = ext_command_t()
        ff.magic = 0xB16B00B5
        ff.node_addr = node
        ff.response = shall_respond
        ff.len = len(frame_data)
        for i in range(0, len(frame_data)):
            ff.data[i] = frame_data[i]
        ff.msg_crc = zlib.crc32(bytes(frame_data))
        ff_bytes = bytes(ff)
        return ff_bytes[0:ff.len+12]

    def send_data(self, node, shall_respond, data):
        ff = self.create_frame(node, shall_respond, data)
        self.ser.write(ff)

    def import_binary_from_bin(self, bin_file):
        with open(bin_file, 'rb') as f:
            return(bytearray(f.read()))

    def import_binary_from_elf(self, elf, virt_addr_start, max_size):
        start = virt_addr_start
        res_words = []
        res_bytes = []
        elffile = ELFFile(open(elf, 'rb'))
        for sec in range(elffile.num_sections()):
            s = elffile.get_section(sec)
            if s.header.sh_addr >= virt_addr_start and s.header.sh_addr + s.header.sh_size <= virt_addr_start + max_size:
                dd = s.data()
                if (len(dd) % 4) != 0:
                    print("Non word aligned segment!!! Error")
                    exit(-1)
                for d in dd: res_bytes.append(d)
                words = unpack('<' + str(int(len(dd)/4)) + 'I', dd)
                for i in range(0, len(words)): res_words.append((s.header.sh_addr + 4 *i, words[i]))

        # Check for spaces
        for i in range(1, len(res_words)):
            if (res_words[i-1][0] != res_words[i][0] - 4):
                print("Found space in data, aborting")
                exit(-1)
        return res_bytes, res_words

    def erase_sector(self, node, addr, size):
        payload = [CMD_SWDL_FLASH_ERASE, 0, 0, 0]
        addr_b = (addr).to_bytes(4, byteorder='little')
        size_b = (size).to_bytes(4, byteorder='little')
        for b in addr_b: payload.append(int(b))
        for b in size_b: payload.append(int(b))
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node did not recognize command")
            print("Error code: " + str(ret['response']))
        else:
            if ret['data'][1] != 0:
                print("Flash erase error: {}".format(ret['data'][1]))

    def flash_seek_address(self, node, addr):
        payload = [CMD_SWDL_FLASH_SEEK, 0, 0, 0]
        addr_b = (addr).to_bytes(4, byteorder='little')
        for b in addr_b: payload.append(int(b))
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node did not recognize command")
            print("Error code: " + str(ret['response']))
        else:
            if ret['data'][1] != 1:
                print("Flash seek error")

    def prepare_node_cache(self, node):
        payload = [CMD_SWDL_PREPARE_CACHE, 0, 0, 0]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node did not recognize command")
            print("Error code: " + str(ret['response']))
        else:
            if ret['data'][1] != 1:
                print("Failed to prepare cache")

    def node_write_to_cache(self, node, data):
        payload = [CMD_SWDL_WRITE_TO_CACHE, 0, 0, 0]
        for d in data: payload.append(d)
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node did not recognize command")
            print("Error code: " + str(ret['response']))
        else:
            if ret['data'][1] != len(data):
                print("Failed to write data to cache")

    def node_write_cache(self, node):
        payload = [CMD_SWDL_WRITE_CACHE, 0, 0, 0]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node did not recognize command")
            print("Error code: " + str(ret['response']))
        else:
            if ret['data'][1] != 1:
                print("Failed to write cache to flash")

    def jump_app(self, node):
        payload = [CMD_JUMP_APP, 0, 0, 0]
        self.send_data(node, 0, payload)

    def jump_boot(self, node):
        payload = [CMD_JUMP_BOOT, 0, 0, 0]
        self.send_data(node, 0, payload)

    def prepare_logging(self, node, wait=False):
        payload = [CMD_PREPARE_LOGGING, 0, 0, 0]
        print('Preparing node {} for logging'.format(node))
        self.send_data(node, 0, payload)
        if wait:
            print('Waiting for node {} to prepare flash'.format(node))
            time.sleep(3)
            while (self.get_sensor_task_time(node) == 0xdeadbeef):
                time.sleep(1)
            print('Log prepared for node {}'.format(node))

    def node_check_crc(self, node, start_addr, data):
        payload = [CMD_CRC_CHECK, 0, 0, 0]
        addr_b = (start_addr).to_bytes(4, byteorder='little')
        size_b = (len(data)).to_bytes(4, byteorder='little')
        for b in addr_b: payload.append(int(b))
        for b in size_b: payload.append(int(b))
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node did not recognize command")
            print("Error code: " + str(ret['response']))
        else:
            data_crc = zlib.crc32(bytes(data))
            calc_crc = int.from_bytes(bytes(ret['data'][4:8]), byteorder='little')
            if ret['data'][1] != 1:
                print("ERROR! Failed to check CRC")
                return None
            if data_crc != calc_crc:
                print('\n\tERROR! Expected CRC: {} differs from targets CRC: {}'.format(hex(data_crc), hex(calc_crc)))
                return None
            return calc_crc

    def seal(self, node, addr, size, crc):
        payload = [CMD_SWDL_SEAL, 0, 0, 0]

        addr_b = (addr).to_bytes(4, byteorder='little')
        size_b = (size).to_bytes(4, byteorder='little')
        crc_b = (crc).to_bytes(4, byteorder='little')

        for b in addr_b: payload.append(int(b))
        for b in size_b: payload.append(int(b))
        for b in crc_b: payload.append(int(b))

        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node did not recognize command")
            print("Error code: " + str(ret['response']))
        else:
            if ret['data'][1] != 0:
                print("Failed to seal app with error code: {}".format(ret['data'][1]))

    def write_firmware(self, node, start_address, firmware_bytes):
        with Progress() as  progress:
            task = progress.add_task("[green] Flashing node {}".format(node), total=len(firmware_bytes))
            full_size = len(firmware_bytes)
            chunks = math.floor(len(firmware_bytes)/256)
            remaining = len(firmware_bytes) % 256

            fb_pointer = 0

            self.dummy = []
            # Seek to start address
            self.flash_seek_address(node, start_address)

            # Write the full chunks
            for i in range(chunks):
                # Prepare the cache
                self.prepare_node_cache(node)
                for j in range(int(256/32)):
                    # Transfer 32 bytes to node cache
                    bytes_to_write = []
                    for k in range(32):
                        bytes_to_write.append(firmware_bytes[fb_pointer])
                        fb_pointer += 1
                    # Send bytes_to_write to cache
                    self.node_write_to_cache(node, bytes_to_write)
                    for b in bytes_to_write: self.dummy.append(b)
                # Write this chunk from cache to flash
                self.node_write_cache(node)
                progress.update(task, advance=256)

            # Prepare the cache
            self.prepare_node_cache(node)
            # Handle the remaining bytes
            remaining_bytes = remaining
            while remaining_bytes > 0:
                if remaining_bytes < 32:
                    # Transfer bytes to cache
                    bytes_to_write = []
                    for k in range(remaining_bytes):
                        bytes_to_write.append(firmware_bytes[fb_pointer])
                        fb_pointer += 1
                    # Send bytes_to_write to cache
                    self.node_write_to_cache(node, bytes_to_write)
                    # Write from cache to flash (node will fill with 0xff to ensure 256 byte alignment)
                    self.node_write_cache(node)
                    for b in bytes_to_write: self.dummy.append(b)
                    progress.update(task, advance=remaining_bytes)
                    remaining_bytes -= remaining_bytes
                else:
                    # Transfer 32 bytes to cache
                    bytes_to_write = []
                    for k in range(32):
                        bytes_to_write.append(firmware_bytes[fb_pointer])
                        fb_pointer += 1
                    # Send to cache
                    self.node_write_to_cache(node, bytes_to_write)
                    for b in bytes_to_write: self.dummy.append(b)
                    progress.update(task, advance=32)
                    remaining_bytes -= 32
                    if remaining_bytes == 0:
                        self.node_write_cache(node)

            for i in range(len(firmware_bytes)):
                if (firmware_bytes[i] != self.dummy[i]):
                    print('Error')

    def swdl(self, binary, node, start_addr=0x10004000, max_size=1024*1024, oneshot=True):
        console.print("[green]Preparing node [/green]{}".format(node))
        prog_bytes = self.import_binary_from_bin(binary)

        if oneshot:
            # FIXME verify that we are in fact in boot
            self.jump_boot(node)
            time.sleep(0.5)

        # Set 100ms erase time
        self.set_i2c_response_delay(100)
        sectors_to_erase = math.ceil(len(prog_bytes)/4096)
        for i in range(sectors_to_erase):
            self.erase_sector(node, start_addr+i*4096, 4096)

        # Packetize and program the words
        self.set_i2c_response_delay(0)
        self.write_firmware(node, start_addr, prog_bytes)
        self.set_i2c_response_delay(100)

        # Check CRC32
        crc = self.node_check_crc(node, start_addr, prog_bytes)

        # Seal the app
        self.seal(node, start_addr, len(prog_bytes), crc)
        crc = self.node_check_crc(node, start_addr, prog_bytes)

        # Profit
        if oneshot:
            self.jump_app(node)
        console.print(":thumbs_up: [green]Node {} flashed [/green]".format(node))

    def set_i2c_response_delay(self, delay):
        payload = [CMD_I2C_RESPONSE_DELAY, 0, 0, 0]
        ms_b = (delay).to_bytes(4, byteorder='little')
        for b in ms_b: payload.append(int(b))
        self.send_data(0, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Failed to set I2C response delay")
            print("Error code: " + str(ret['response']))

    def set_controller_state(self, state):
        if state:
            payload = [CMD_CONTROLLER_START, 0, 0, 0]
        else:
            payload = [CMD_CONTROLLER_STOP, 0, 0, 0]
        self.send_data(0, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Failed to set controller state")
            print("Error code: " + str(ret['response']))

    def print_msg(self, msg, direction):
        print('')
        if direction == 'rx':
            print('=====| RX Message |=====')
        print('Magic: {} CRC32: {} Node: {} len: {}'.format(hex(msg['magic']), hex(msg['msg_crc']), msg['node_addr'], msg['len']))
        print('Message Error: {}'.format(msg['response']))
        cmd = msg['data'][0]
        if cmd in protocol_cmd_t__enumvalues:
            print('Command: {}'.format(protocol_cmd_t__enumvalues[msg['data'][0]]))
        print('Raw Data:')
        print('[')
        for d in msg['data']:
            print('    {}'.format(hex(d)))
        print(']')

    def get_app_version(self, node):
        payload = [CMD_REQ_APP_VERSION]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        version = int.from_bytes(ret['data'], byteorder='little')
        console.print("[green]App version: [/green]{}".format(hex(version)))

    def get_sensor_task_time(self, node):
        payload = [CMD_REQ_NODE_TASK_TIME]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        task_time_us = int.from_bytes(ret['data'], byteorder='little')
        return task_time_us

    def settings_set(self, node, id, value):
        payload = [CMD_SET_SETTING, id, 0, 0]
        value_b = (value).to_bytes(4, byteorder='little')
        for b in value_b: payload.append(int(b))
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        res = ret['data'][0]
        if ret['response'] != 0:
            print("Error! Node did not recognize settings command")
        if (res != 1):
            print('Failed to set parameter {} for node {}'.format(id, value))

    def settings_set_all_nodes(self, id, value):
        for i in range(10):
            self.settings_set(i+1, id, value)

    def start_logging(self, node):
        print('Starting logging for node {}'.format(node))
        payload = [CMD_START_LOGGING]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node failed to start logging")
            print("Error code: " + str(ret['response']))

    def start_ram_logging(self, node):
        payload = [CMD_START_RAM_LOGGING]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node failed to start RAM logging")
            print("Error code: " + str(ret['response']))

    def stop_ram_logging(self, node):
        payload = [CMD_STOP_RAM_LOGGING]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node failed to stop RAM logging")
            print("Error code: " + str(ret['response']))

    def read_log_data(self, node, packets, from_ram=False):
        up = []
        left = []
        right = []
        down = []
        self.set_i2c_response_delay(0)
        offset = 0
        with Progress() as  progress:
            task = progress.add_task("[green] Reading log from node {}".format(node), total=packets)
            while packets >= 6:
                read_packets = self.read_log_data_packets(node, offset, from_ram)
                for i in read_packets['packets']:
                    d = getdict(i)

                    up.append(d['up'])
                    left.append(d['left'])
                    right.append(d['right'])
                    down.append(d['down'])

                packets -= 6
                if from_ram:
                    offset += 6
                else:
                    offset += 48
                progress.update(task, advance=6)

        if from_ram:
            up.reverse()
            left.reverse()
            right.reverse()
            down.reverse()

        return {'up': up, 'left': left, 'right': right, 'down': down}

    def write_log_data(self, log_data, filename):
        pickle.dump(log_data, open(filename, 'wb'))

    def read_log_data_packets(self, node, offset, from_ram):
        if from_ram:
            payload = [CMD_REQ_RAM_LOG_DATA, 0, 0, 0]
        else:
            payload = [CMD_REQ_LOG_DATA, 0, 0, 0]
        offset_b = (offset).to_bytes(4, byteorder='little')
        for b in offset_b: payload.append(int(b))
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        if ret['response'] != 0:
            print("Error! Node failed to start logging")
            print("Error code: " + str(ret['response']))
        bb = bytes(ret['data'])
        res_struct = sensor_log_data_transfer_packet_t()
        memmove(pointer(res_struct), bb, sizeof(res_struct))
        return getdict(res_struct)

    def stop_logging(self, node):
        print("Stop logging for node {}".format(node))
        payload = [CMD_STOP_LOGGING]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        packets_written = int.from_bytes(ret['data'], byteorder='little')
        if ret['response'] != 0:
            return -1
        else:
            return packets_written

    def get_node_error(self, node):
        payload = [CMD_REQ_NODE_ERROR]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        bb = bytes(ret['data'])
        error = int.from_bytes(bb[4:], byteorder='little')
        if error != NODE_ERROR_NONE:
            print(node_error_t__enumvalues[error])

    def save_calibration(self, node):
        payload = [CMD_SAVE_CALIBRATION]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        self.print_msg(ret, 'rx')

    def get_calibration(self, node, shall_print=True):
        payload = [CMD_REQ_CALIBRATION]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        bb = bytes(ret['data'])
        res_struct = calibration_t()
        memmove(pointer(res_struct), bb, sizeof(res_struct))
        drs = getdict(res_struct)
        if shall_print:
            print('-- Mag_idle:')
            for i in range(0, len(drs['mag_idle'])):
                print(drs['mag_idle'][i])
            print('-- Mag_max:')
            for i in range(0, len(drs['mag_max'])):
                print(drs['mag_max'][i])
            print('-- Acc_idle:')
            print(drs['acc_idle'])
            print('-- Acc_max:')
            print(drs['acc_max'])
        return drs

    def get_stream_data(self, node):
        payload = [CMD_REQ_STREAM_DATA]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        bb = bytes(ret['data'])
        res_struct = stream_data_t()
        memmove(pointer(res_struct), bb, sizeof(res_struct))
        return getdict(res_struct)

    def get_sample_stream(self, node):
        payload = [CMD_REQ_NODE_SAMPLE]
        self.send_data(node, 1, payload)
        ret = self.read_msg()
        bb = bytes(ret['data'])
        res_struct = node_sample_t()
        memmove(pointer(res_struct), bb, sizeof(res_struct))
        return getdict(res_struct)

    def get_all_nodes_samples(self):
        payload = [CMD_CONTROLLER_REQ_NODE_DATA]
        self.send_data(0, 1, payload)
        ret = self.read_msg()
        bb = bytes(ret['data'])
        res_struct = node_reports_t()
        memmove(pointer(res_struct), bb, sizeof(res_struct))
        return res_struct

    def get_all_nodes_errors(self):
        payload = [CMD_CONTROLLER_REQ_NODE_ERROR]
        self.send_data(0, 1, payload)
        ret = self.read_msg()
        bb = bytes(ret['data'])
        res_struct = node_errors_t()
        memmove(pointer(res_struct), bb, sizeof(res_struct))
        return res_struct

    def calibrate(self, start_node, end_node):
        self.set_i2c_response_delay(100)

        input("*Set nodes {}->{} to idle position and press enter...".format(start_node, end_node))

        for i in range(start_node, end_node+1):
            payload = [CMD_START_MAG_IDLE_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        input("*Calibrating idle, press enter to complete..")

        for i in range(start_node, end_node+1):
            payload = [CMD_END_MAG_IDLE_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        input("*Idle complete, press enter to start calibrating endpoints...")

        for i in range(start_node, end_node+1):
            payload = [CMD_START_MAG_ENDPOINTS_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        print("\n\n\t Move sticks for nodes {}->{} to their endpoints!\n\n".format(start_node, end_node))
        input("*Press enter to finish endpoints calibration...")

        for i in range(start_node, end_node+1):
            payload = [CMD_END_MAG_ENDPOINTS_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        print("*Mag calibration done, next up IMU..")
        input("*Set nodes {}->{} to idle position and press enter...".format(start_node, end_node))

        for i in range(start_node, end_node+1):
            payload = [CMD_START_ACC_IDLE_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        input("*Calibrating idle, press enter to complete..")

        for i in range(start_node, end_node+1):
            payload = [CMD_END_ACC_IDLE_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        input("*Idle complete, press enter to start calibrating endpoints...")

        for i in range(start_node, end_node+1):
            payload = [CMD_START_ACC_ENDPOINTS_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        print("\n\n\t Bang sticks for nodes {}->{} to their endpoints!\n\n".format(start_node, end_node))
        input("*Press enter to finish endpoints calibration...")

        for i in range(start_node, end_node+1):
            payload = [CMD_END_ACC_ENDPOINTS_CALIBRATION]
            self.send_data(i, 1, payload)
            ret = self.read_msg()

        print("*Calibration done:")

        for i in range(start_node, end_node+1):
            self.save_calibration(i)
            self.get_calibration(i)

    def sample_all_nodes_with_time_measurment(self):
        st = time.time()
        samples = self.get_all_nodes_samples().samples

        self.error_read_counter += 1
        if self.error_read_counter >= 100:
            errors = self.get_all_nodes_errors().errors
            for i in range(0, len(errors)):
                self.errors[i] = errors[i]
            self.error_read_counter = 0
        et = time.time()

        delta = et - st

        return {'samples': samples, 'errors': self.errors, 'delta': delta}

def main(
        command,
        port = typer.Option(..., help='Magslide port to connect to'),
        nodes = typer.Option('', help='Nodes for operation'),
        log_dir = typer.Option('', help='Nodes for operation'),
        analyze_sensor = typer.Option('', help='Specify (node:sensor) a specific sensor shall be analyzed'),
        firmware = typer.Option('', help='Firmware for swdl')
    ):
    global sim

    Art = text2art("Magslide Toolbox\nVer. 0.1", font='tarty1')
    console.print('[green]' + Art + '[/green]')
    console.print(':computer: Running command: [bold green]' + command + '[/bold green]')
    controller = MagslideController(port)

    if command == 'swdl_node':
        controller.connect()
        controller.set_controller_state(False)
        nsp = nodes.split('-')
        start_node = int(nsp[0])
        end_node = int(nsp[1])

        for i in range(start_node, end_node+1):
            controller.swdl(firmware, i)
            time.sleep(0.2)
            controller.get_app_version(i)
        controller.set_i2c_response_delay(0)
        controller.set_controller_state(True)

    elif command == 'calibrate':
        controller.connect()
        nsp = nodes.split('-')
        start_node = int(nsp[0])
        end_node = int(nsp[1])
        controller.calibrate(start_node, end_node)

    elif command == 'get_calibration':
        node = int(nodes)
        controller.connect()
        controller.get_calibration(node)

    ##################################################
    # The combined target for a swdl for the controller
    ##################################################
    elif command == 'swdl_controller':
        # Jump to boot
        # FIXME sort out arguments (probably broken now.. after cleanup (shall use --port etc.))
        os.system('python {} {} {} {}'.format(sys.argv[0], command + '_do_jump_boot', sys.argv[2], firmware))
        found = await_reset(controller.port)
        os.system('python {} {} {} {}'.format(sys.argv[0], command + '_do_swdl', sys.argv[2], firmware))
        found = await_reset(controller.port)
        time.sleep(0.5)
        controller.connect()
        controller.get_app_version(0)

    # Part 1
    elif command == 'swdl_controller_do_jump_boot':
        controller.connect()
        controller.jump_boot(0)

    # Part 2
    elif command == 'swdl_controller_do_swdl':
        controller.connect()
        controller.swdl(firmware, 0, start_addr=0x10006000, oneshot=False)
        controller.jump_app(0)
    ##################################################

    elif command == 'measure_sensors_task':
        controller.connect()
        controller.set_i2c_response_delay(100)
        print('{}us'.format(controller.get_sensor_task_time(1)))

    elif command == 'flash_log':
        nsp = nodes.split('-')
        start_node = int(nsp[0])
        end_node = int(nsp[1])

        controller.connect()
        controller.set_i2c_response_delay(100)

        for i in range(start_node, end_node):
            controller.prepare_logging(i)
        controller.prepare_logging(end_node, wait=True)

        controller.set_i2c_response_delay(0)

        # Verify that all nodes are ok
        all_ok = False
        while not all_ok:
            all_ok = True
            for i in range(start_node, end_node+1):
                if controller.get_sensor_task_time(i) == 0xdeadbeef:
                    all_ok = False

        print('All nodes are prepared for logging')

        log_name = input("*Enter log name and press enter to start logging: ")
        Path("./_output/{}".format(log_name)).mkdir(parents=True, exist_ok=True)

        st = time.time()

        for i in range(start_node, end_node+1):
            controller.start_logging(i)

        input("*Logging, press enter to stop..")

        controller.set_i2c_response_delay(10)
        et = time.time()
        node_packets = {}
        for i in range(start_node, end_node+1):
            retries = 4
            while retries > 0:
                packets_written = controller.stop_logging(i)
                if packets_written == -1:
                    retries -= 1
                    print('Failed to stop logging on node: {}, retrying..'.format(i))
                    time.sleep(0.1)
                else:
                    # We got valid packets
                    break

            print('{} packets written ({}kb) in {} seconds'.format(packets_written, (packets_written*8)/1024, round(et-st, 2)))
            node_packets["{}".format(i)] = packets_written

        for i in range(start_node, end_node+1):
            res = controller.read_log_data(i, node_packets["{}".format(i)])
            controller.write_log_data(res, '_output/{}/node_{}.data'.format(log_name, i))

        metadata = {}
        for i in range(start_node, end_node+1):
            res = controller.get_calibration(i, shall_print=False)
            cal = {}

            cal['up_idle'] = int(res['mag_idle'][1])
            cal['up_max'] = int(res['mag_max'][1])

            cal['right_idle'] = int(res['mag_idle'][0])
            cal['right_max'] = int(res['mag_max'][0])

            cal['left_idle'] = int(res['mag_idle'][2])
            cal['left_max'] = int(res['mag_max'][2])

            cal['down_idle'] = int(res['mag_idle'][3])
            cal['down_max'] = int(res['mag_max'][3])

            metadata["calibration_node_{}".format(i)] = cal

        # Add sample rate as a metadata field?
        controller.write_log_data(metadata, '_output/{}/metadata.data'.format(log_name))
        print('\n*All logs are written\n')

    elif command == 'run':
        os.system('cd node/x86_sil/build && make -j16')

        controller.connect()
        controller.set_i2c_response_delay(0)

        for i in range(10):
            controller.start_ram_logging(i+1)

        print('Task times:')
        for i in range(10):
            print('{}us'.format(controller.get_sensor_task_time(i+1)))

        settings = Settings.settings
        for k in settings:
            s = settings[k]
            controller.settings_set_all_nodes(s['id'], s['default'])
            s['current_value'] = s['default']

        app = QApplication(sys.argv)
        q = queue.Queue()
        widget = MagslideGUI(settings, q)
        pt = Pollthread(controller, q)
        widget.connect_magic(pt)
        pt.start()
        app.exec_()

        pt.stop()

        if hasattr(widget, 'shall_sil'):
            time.sleep(0.1)
            written_text = widget.te.toPlainText()
            print('Written text: \n\n{}\n\n'.format(written_text))
            settings = widget.settings
            print('Starting simulation')

            ms_to_read = 10000
            log_name = 'sil'
            print('\n\n*Starting simulation on the previous {}ms..'.format(ms_to_read))

            Path("./_output/{}".format(log_name)).mkdir(parents=True, exist_ok=True)

            # Stop the logging simultaniously
            for i in range(10):
                controller.stop_ram_logging(i+1)

            for i in range(10):
                res = controller.read_log_data(i+1, ms_to_read, from_ram=True)
                controller.write_log_data(res, '_output/{}/node_{}.data'.format(log_name, i+1))

            for i in range(10):
                controller.start_ram_logging(i+1)

            sim = MagslideSIL(data_dir='_output/{}'.format(log_name))
            sim.setup(settings)
            sim.run()

    elif command == 'simulate':
        os.system('cd node/x86_sil/build && make -j16')
        # Use default settings for SIL
        settings = Settings.settings
        for k in settings:
            s = settings[k]
            s['current_value'] = s['default']
        sim = MagslideSIL(data_dir=log_dir, analyze_sensor=analyze_sensor)
        sim.setup(settings)
        sim.run()

    elif command == 'test':
        # Test command to use under development
        controller.connect()
        controller.set_i2c_response_delay(0)
        controller.settings_set_all_nodes(SETTING_Y_THRESH, 1000)

    else:
        print("Error wrong command: {}".format(command))

# Create one cb function for each node if needed (i.e 10)
def sil_sensor_sample_cb(data, offset):
    global sim
    pass

if __name__ == "__main__":
    typer.run(main)
