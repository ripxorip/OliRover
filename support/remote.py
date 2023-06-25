import sys
import os
import typing
from PyQt5 import QtGui

from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QGroupBox, QRadioButton, QSlider, QVBoxLayout, QHBoxLayout, QTextEdit, QLineEdit
from PyQt5.QtCore import QObject, pyqtSlot, QThread, pyqtSignal, Qt

class Remote(QWidget):
    def __init__(self):
        super().__init__()

        self.keyPressEvent = self.keyPressEvent

        grid = QGridLayout()
        self.setLayout(grid)

        self.setWindowTitle("Robot Remote Controller")
        self.resize(400, 200)
        self.show()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        print("Pressed")
        if event.key() == Qt.Key_Left:
            print('Left')
        elif event.key() == Qt.Key_Right:
            print('Right')
        elif event.key() == Qt.Key_Up:
            print('Up')
        elif event.key() == Qt.Key_Down:
            print('Down')
        print('\n\n')

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        print("Released")
        if event.key() == Qt.Key_Left:
            print('Left')
        elif event.key() == Qt.Key_Right:
            print('Right')
        elif event.key() == Qt.Key_Up:
            print('Up')
        elif event.key() == Qt.Key_Down:
            print('Down')
        print('\n\n')

def main():
    app = QApplication(sys.argv)
    r = Remote()
    app.exec_()

if __name__ == '__main__':
    main()
    print('Done')