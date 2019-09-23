# -*- coding: utf-8 -*-
"""
Created on Sat Aug  3 15:25:57 2019
IREC Drone code
@author: saidm

Simple code to test the throw mode function
"""
#################### Setup Begin ##########################
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
#from queue import Queue 
#from threading import Thread
import time
import serial
#usbCom = serial.Serial('/dev/ttyACM0', 9600)
#usbCom.open()

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

##################### Setup End ##############################

#Test throw mode
#Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)
vehicle.mode = VehicleMode("THROW", THROW_TYPE=1, THROW_MOT_START=0, THROW_NEXTMODE="LAND")
vehicle.armed = True

# Close vehicle object
vehicle.armed = False
vehicle.close()
