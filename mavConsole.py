#!/usr/bin/env python3


from include.mavros import MavROS
from include.interface import Interface

if __name__ == '__main__':
    global interface, mavros
    interface = Interface()
    mavros = MavROS(interface)
    interface.uiApp.exec()

