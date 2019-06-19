#!/usr/bin/env python
import rospy
import odrive
from odrive.enums import *
from time import sleep
import math


RED='\033[0;31m'
NC='\033[0m' # No Color
GREEN='\033[0;32m'
YELLOW='\033[0;33m'

unstandart_hall_timigs = 0

odrv0 = odrive.find_any()
print(odrv0.vbus_voltage)


# Left motor
odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 25 #Requires config save and reboot
odrv0.axis0.motor.config.current_control_bandwidth = 100

odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90

odrv0.axis0.encoder.config.bandwidth = 100
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.1
odrv0.axis0.controller.config.vel_limit = 1000
odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

# Right motor
odrv0.axis1.motor.config.pole_pairs = 15
odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
odrv0.axis1.motor.config.requested_current_range = 25 #Requires config save and reboot
odrv0.axis1.motor.config.current_control_bandwidth = 100

odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis1.encoder.config.cpr = 90

odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.controller.config.pos_gain = 1
odrv0.axis1.controller.config.vel_gain = 0.02
odrv0.axis1.controller.config.vel_integrator_gain = 0.1
odrv0.axis1.controller.config.vel_limit = 1000
odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

odrv0.save_configuration()

try:
    odrv0.reboot()
except Exception:
    pass
sleep(1)

odrv0 = odrive.find_any()

# Left motor calibration
print("\n\n{}Left motor calibration (axis0){}".format(GREEN, NC))
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION # Start motor calibration
sleep(10) # Wait till the end of calibration sequence
print("\n{}Motor calibration parameters{}".format(YELLOW, NC))
print(odrv0.axis0.motor) # Print motor calibration parameters

odrv0.axis0.motor.config.pre_calibrated = True # Save motor calibration parameters

sleep(5) # Wait till all calibrated motor parameters are saved

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION # Start encoder calibration
sleep(10)  # Wait till the end of calibration sequence
enc_param = odrv0.axis0.encoder
print("\n{}Hall sensor calibration parameters{}".format(YELLOW, NC))
print(odrv0.axis0.encoder) # Print encoder calibration parameters

if(not(0.4 < math.fabs(enc_param.config.__getattribute__('offset_float')) < 0.6) and not(unstandart_hall_timigs)):
    print("{}WARNING{}".format(RED, NC))
    print("It seems that you motor hall sensors use not stantart timings or something went wrong")
    print('Your offset_float parameter is:{}'.format(enc_param.config.__getattribute__('offset_float')))
    print('Your calibration parameters was not saved. '
          'If you are sure that there are no mistake run calibration again with flag unstandart_hall_timigs')
else:
    odrv0.axis0.encoder.config.pre_calibrated = True
    sleep(5)  # Wait till all calibrated encoder parameters are saved


# Right motor calibration
print("\n\n{}Right motor calibration (axis1){}".format(GREEN, NC))
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION # Start motor calibration
sleep(10) # Wait till the end of calibration sequence
print("\n{}Motor calibration parameters{}".format(YELLOW, NC))
print(odrv0.axis1.motor) # Print motor calibration parameters

odrv0.axis1.motor.config.pre_calibrated = True # Save motor calibration parameters

sleep(5) # Wait till all calibrated motor parameters are saved

odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION # Start encoder calibration
sleep(10)  # Wait till the end of calibration sequence
enc_param = odrv0.axis1.encoder
print("\n{}Hall sensor calibration parameters{}".format(YELLOW, NC))
print(odrv0.axis1.encoder) # Print encoder calibration parameters

if(not(0.4 < math.fabs(enc_param.config.__getattribute__('offset_float')) < 0.6) and not(unstandart_hall_timigs)):
    print("{}WARNING{}".format(RED, NC))
    print("It seems that you right motor hall sensors use not stantart timings or something went wrong")
    print('Your offset_float parameter is:{}'.format(enc_param.config.__getattribute__('offset_float')))
    print('Your calibration parameters was not saved. '
          'If you are sure that there are no mistake run calibration again with flag unstandart_hall_timigs')
else:
    odrv0.axis1.encoder.config.pre_calibrated = True
    sleep(5)  # Wait till all calibrated encoder parameters are saved

odrv0.save_configuration()

try:
    odrv0.reboot()
except Exception:
    pass
sleep(1)
odrv0 = odrive.find_any()

#Parameters check
print("\n\n{}Left wheel settings{}".format(GREEN, NC))
print(odrv0.axis0)
print("\n\n{}Right wheel settings{}".format(GREEN, NC))
print(odrv0.axis1)

# Test close loop control
print("\n\n{}Testing of performed calibration{}".format(GREEN, NC))
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.vel_setpoint = 120
odrv0.axis1.controller.vel_setpoint = 120
sleep(5)
# Your motor should spin here
odrv0.axis0.controller.vel_setpoint = 0
odrv0.axis1.controller.vel_setpoint = 0
odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis1.requested_state = AXIS_STATE_IDLE

