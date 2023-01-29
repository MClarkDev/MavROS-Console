#!/usr/bin/env python3

import sys
import time
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton, QLabel, QHBoxLayout, QGroupBox
from PyQt5.QtCore import QTimer

from include.state import AppState

class Interface:

    state = AppState()

    uiApp = QApplication([])
    uiWin = QWidget()

    uiTimer = QTimer()

    uiAddr = QLabel("-")
    uiConn = QPushButton("Connect")

    uiStat = QLabel("-")
    uiMode = QLabel("-")
    uiGuid = QLabel("-")

    uiArmd = QLabel("-")
    uiArm0 = QPushButton("Disarm")
    uiArm0.setEnabled(False)
    uiArm1 = QPushButton("Arm")
    uiArm1.setEnabled(False)

    uiLast = QLabel("-")
    uiBatV = QLabel("-")
    uiBatA = QLabel("-")

    uiImuX = QLabel("-")
    uiImuY = QLabel("-")
    uiImuZ = QLabel("-")

    uiGpsLoc = QLabel("-")
    uiGpsAlt = QLabel("-")
    uiGpsCnt = QLabel("-")
    uiGpsHdg = QLabel("-")

    def __init__(self):

        layout = QGridLayout()

        g0 = QGroupBox("Connection")
        header = QGridLayout()
        g0.setLayout(header)
        layout.addWidget(g0, 0, 0, 1, 4)
        header.addWidget(self.uiAddr, 0, 0, 1, 2)
        header.addWidget(self.uiConn, 0, 2, 1, 1)

        g1 = QGroupBox("Status")
        status = QGridLayout()
        g1.setLayout(status)
        layout.addWidget(g1, 1, 0, 1, 2)
        status.addWidget(QLabel("Status"), 0, 0, 1, 1)
        status.addWidget(self.uiStat, 0, 1, 1, 1)
        status.addWidget(QLabel("Mode"), 1, 0, 1, 1)
        status.addWidget(self.uiMode, 1, 1, 1, 1)
        status.addWidget(QLabel("Guided"), 2, 0, 1, 1)
        status.addWidget(self.uiGuid, 2, 1, 1, 1)

        g2 = QGroupBox("Arming")
        arming = QGridLayout()
        g2.setLayout(arming)
        layout.addWidget(g2, 1, 2, 1, 2)
        arming.addWidget(QLabel("Armed"), 0, 0, 1, 1)
        arming.addWidget(self.uiArmd, 0, 1, 1, 1)
        arming.addWidget(self.uiArm0, 1, 0, 1, 1)
        arming.addWidget(self.uiArm1, 1, 1, 1, 1)

        g3 = QGroupBox("Health")
        health = QGridLayout()
        g3.setLayout(health)
        layout.addWidget(g3, 2, 0, 1, 2)
        health.addWidget(QLabel("Last"), 0, 0, 1, 1)
        health.addWidget(self.uiLast, 0, 1, 1, 1)
        health.addWidget(QLabel("Volts"), 1, 0, 1, 1)
        health.addWidget(self.uiBatV, 1, 1, 1, 1)
        health.addWidget(QLabel("Amps"), 2, 0, 1, 1)
        health.addWidget(self.uiBatA, 2, 1, 1, 1)

        g4 = QGroupBox("IMU")
        imu = QGridLayout()
        g4.setLayout(imu)
        layout.addWidget(g4, 2, 2, 1, 2)
        imu.addWidget(QLabel("X:"), 2, 0, 1, 1)
        imu.addWidget(self.uiImuX, 2, 1, 1, 1)
        imu.addWidget(QLabel("Y:"), 3, 0, 1, 1)
        imu.addWidget(self.uiImuY, 3, 1, 1, 1)
        imu.addWidget(QLabel("Z:"), 4, 0, 1, 1)
        imu.addWidget(self.uiImuZ, 4, 1, 1, 1)

        g5 = QGroupBox("GPS")
        gps = QGridLayout()
        g5.setLayout(gps)
        layout.addWidget(g5, 3, 0, 1, 4)
        gps.addWidget(QLabel("Location"), 0, 0, 1, 1)
        gps.addWidget(self.uiGpsLoc, 0, 1, 1, 2)
        gps.addWidget(QLabel("Satellites"), 0, 3, 1, 1)
        gps.addWidget(self.uiGpsCnt, 0, 4, 1, 1)

        gps.addWidget(QLabel("Altitude"), 1, 0, 1, 1)
        gps.addWidget(self.uiGpsAlt, 1, 1, 1, 2)
        gps.addWidget(QLabel("Heading"), 1, 3, 1, 1)
        gps.addWidget(self.uiGpsHdg, 1, 4, 1, 1)

        self.uiTimer.timeout.connect(self.updateUI)
        self.uiTimer.setInterval(100)

        self.uiWin.setLayout(layout)
        self.uiWin.show()

    def updateUI(self):
        diff = time.time() - self.state.last
        self.uiLast.setText("{:.1f}s".format(diff))
        self.uiImuX.setText("{:.3f}".format(self.state.imuX))
        self.uiImuY.setText("{:.3f}".format(self.state.imuY))
        self.uiImuZ.setText("{:.3f}".format(self.state.imuZ))
        self.uiMode.setText(self.state.stateMode)
        self.uiStat.setText(self.state.stateMap[self.state.stateStat])
        self.uiGuid.setText(str(self.state.stateGuid))
        self.uiArmd.setText(str(self.state.stateArmd))
        self.uiArm0.setEnabled(self.state.stateArmd)
        self.uiArm1.setEnabled(not self.state.stateArmd)
        self.uiBatV.setText("{:.2f}v".format(self.state.pwrV))
        self.uiBatA.setText("{:.2f}A".format(abs(self.state.pwrA)))
        self.uiGpsLoc.setText("{:.4f}, {:.4f}".format(self.state.gpsLat, self.state.gpsLon))
        self.uiGpsAlt.setText("{:.2f} feet".format(self.state.gpsAlt))
        self.uiGpsCnt.setText(str(self.state.gpsCnt))
        self.uiGpsHdg.setText(str(self.state.gpsHdg))

