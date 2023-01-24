#!/usr/bin/env python3

import os
import sys
import time
import rospy

from include.interface import Interface

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
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
        self.interface.uiConn.setEnabled(False)
        rospy.Subscriber('/mavros/state', State, self.dataCallbackState)
        rospy.Subscriber('/mavros/imu/data', Imu, self.dataCallbackImu)
        self.svcArming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    def actionArm(self, unknown):
        self.svcArming(True)

    def actionDisarm(self, unknown):
        self.svcArming(False)

    def dataCallbackImu(self, data):
        self.interface.setImuData(data.orientation.x, data.orientation.y, data.orientation.z)

    def dataCallbackState(self, data):
        self.interface.setStateData(data.mode, data.armed)

