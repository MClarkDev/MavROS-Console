#!/usr/bin/env python3

class AppState:

    last = 0

    imuX = 0
    imuY = 0
    imuZ = 0

    stateMode = 0
    stateArmd = 0
    stateGuid = 0

    pwrV = 0
    pwrA = 0

    gpsCnt = 0
    gpsLat = 0
    gpsLon = 0
    gpsAlt = 0
    gpsHdg = 0

    def __init__(self):
        return
