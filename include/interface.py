#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton, QLabel, QHBoxLayout

class Interface:

    uiApp = QApplication([])
    uiWin = QWidget()

    uiAddr = QLabel("-")
    uiConn = QPushButton("Connect")

    uiArmd = QLabel("-")
    uiArm0 = QPushButton("Disarm")
    uiArm0.setEnabled(False)
    uiArm1 = QPushButton("Arm")
    uiArm1.setEnabled(False)
    uiMode = QLabel("-")


    uiImuX = QLabel("-")
    uiImuY = QLabel("-")
    uiImuZ = QLabel("-")

    def __init__(self):

        layout = QGridLayout()

        header = QHBoxLayout()
        header.addWidget(QLabel("Address"))
        header.addWidget(self.uiAddr)
        header.addWidget(self.uiConn)
        layout.addLayout(header, 0, 0, 1, 4)

        status = QGridLayout()
        status.addWidget(QLabel("Armed"), 0, 0, 1, 1)
        status.addWidget(self.uiArmd, 0, 1, 1, 1)
        status.addWidget(self.uiArm0, 0, 3, 1, 1)
        status.addWidget(self.uiArm1, 0, 4, 1, 1)
        status.addWidget(QLabel("Mode"), 1, 0, 1, 1)
        status.addWidget(self.uiMode, 1, 1, 1, 1)
        layout.addLayout(status, 1, 0, 1, 4)

        layout.addWidget(QLabel("X:"), 2, 0, 1, 1)
        layout.addWidget(self.uiImuX, 2, 1, 1, 1)
        layout.addWidget(QLabel("Y:"), 3, 0, 1, 1)
        layout.addWidget(self.uiImuY, 3, 1, 1, 1)
        layout.addWidget(QLabel("Z:"), 4, 0, 1, 1)
        layout.addWidget(self.uiImuZ, 4, 1, 1, 1)

        self.uiWin.setLayout(layout)
        self.uiWin.show()

    def setImuData(self, x, y, z):
        self.uiImuX.setText("{:.3f}".format(x))
        self.uiImuY.setText("{:.3f}".format(y))
        self.uiImuZ.setText("{:.3f}".format(z))

    def setStateData(self, mode, armed):
        self.uiMode.setText(mode)
        self.uiArmd.setText(str(armed))
        self.uiArm0.setEnabled(armed)
        self.uiArm1.setEnabled(not armed)

