# -*- coding: utf-8 -*-
"""
Created on Sat Aug  3 15:25:57 2019
IREC Drone code
@author: saidm

Draft of code used for full test
"""

#################### Setup Begin ##########################
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
#from queue import Queue 
#from threading import Thread
import time
import serial
usbCom = serial.Serial('/dev/ttyACM0', 9600)
usbCom.open()

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

######################## Setup End ##############################

# function to prepare for landing drone
def descending():
	vehicle.armed = False
	last_alt = 0
	current_alt = 0
	# check for descending to ~400ft
	while True:
		current_alt = vehicle.location.global_relative_frame.alt
		if current_alt < last_alt and current_alt <= 400:
			break
		last_alt = current_alt
	# prepare to land
	throw_mode_func()

# set to throw mode and land after stabilizing
def throw_mode_func():
	# set to throw mode and arm vehicle 
	vehicle.mode = VehicleMode("THROW", THROW_TYPE=1, THROW_MOT_START=0, THROW_NEXTMODE="LAND")
	vehicle.armed = True
	
	while vehicle.mode != "THROW":
		pass
	# turn servo to detach from parachute
	usbCom.flushInput()
	usbCom.write('1') 
	usbCom.flush()
	
# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: "+str(vehicle.location.global_relative_frame.alt))
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)

def data_collect():
	usbCom.write('2') ## Start data collection


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

descending()
data_collect()
arm_and_takeoff(20)
	
