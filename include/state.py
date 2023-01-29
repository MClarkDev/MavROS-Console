#!/usr/bin/env python3

class AppState:

    last = 0

    imuRx = 0
    imuRy = 0
    imuRz = 0

    imuAx = 0
    imuAy = 0
    imuAz = 0

    stateMode = ""
    stateArmd = 0
    stateGuid = 0
    stateStat = 0

    pwrV = 0
    pwrA = 0

    gpsCnt = 0
    gpsLat = 0
    gpsLon = 0
    gpsAlt = 0
    gpsHdg = 0

    stateMap = [
        "Unknown", "Booting", "Calibrating",
        "Standby", "Active", "Critical",
        "Emergency", "PowerDown", "Termination"
    ]

    def __init__(self):
        return
