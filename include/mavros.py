#!/usr/bin/env python3

import os
import sys
import time
import rospy

from include.state import AppState
from include.interface import Interface

from std_msgs.msg import Bool, UInt32, Float64
from sensor_msgs.msg import Imu, BatteryState, NavSatFix
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State

class MavROS:

    interface = None

    svcArming = None

    def __init__(self, interface):
        rospy.init_node('mavConsole', anonymous=True)

        self.interface = interface;
        self.interface.uiAddr.setText(os.getenv("ROS_MASTER_URI"))
        self.interface.uiConn.clicked.connect(self.actionConnect)
        self.interface.uiArm0.clicked.connect(self.actionDisarm)
        self.interface.uiArm1.clicked.connect(self.actionArm)

    def actionConnect(self, unknown):
        self.interface.uiTimer.start()
        self.interface.uiConn.setEnabled(False)
        rospy.Subscriber('/mavros/state', State, self.dataCallbackState)
        rospy.Subscriber('/mavros/battery', BatteryState, self.dataCallbackBattery)
        rospy.Subscriber('/mavros/imu/data', Imu, self.dataCallbackImu)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.dataCallbackLocation)
        rospy.Subscriber('/mavros/global_position/raw/satellites', UInt32, self.dataCallbackSatellites)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.dataCallbackHeading)
        self.svcArming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    def actionArm(self, unknown):
        self.svcArming(True)

    def actionDisarm(self, unknown):
        self.svcArming(False)

    def dataCallbackImu(self, data):
        self.interface.state.last = time.time()
        self.interface.state.imuX = data.orientation.x
        self.interface.state.imuY = data.orientation.y
        self.interface.state.imuZ = data.orientation.z

    def dataCallbackState(self, data):
        self.interface.state.last = time.time()
        self.interface.state.stateMode = data.mode
        self.interface.state.stateArmd = data.armed
        self.interface.state.stateGuid = data.guided

    def dataCallbackBattery(self, data):
        self.interface.state.last = time.time()
        self.interface.state.pwrV = data.voltage
        self.interface.state.pwrA = data.current

    def dataCallbackLocation(self, data):
        self.interface.state.last = time.time()
        self.interface.state.gpsLat = data.latitude
        self.interface.state.gpsLon = data.longitude
        self.interface.state.gpsAlt = data.altitude

    def dataCallbackSatellites(self, data):
        self.interface.state.last = time.time()
        self.interface.state.gpsCnt = data.data

    def dataCallbackHeading(self, data):
        self.interface.state.last = time.time()
        self.interface.state.gpsHdg = data.data

