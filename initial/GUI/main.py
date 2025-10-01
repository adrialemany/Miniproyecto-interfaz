import sys

from PyQt5.QtCore import QSize, Qt

from PyQt5.QtWidgets import QMainWindow, QApplication

sys.path.append("./..") # potential field stuff

from obstacle import Obstacle
from robot import Robot
from geom_utils import *
from potentialFieldPathPlanner import PotentialFieldPathPlanner

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Path planning with Artificial Potential Fields")
        self.setFixedSize(600,600)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    app.exec()