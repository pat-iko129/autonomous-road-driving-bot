#!/usr/bin/env python3
#
#   behavior.py
#   
#   This includes the line following and turning behaviors.

# Imports
import pigpio
import sys
import time
import traceback
import motor
import drivesystem
import linesensor
import filter
import map
import intersection as inter
import pickle

import matplotlib.pyplot as plt
import numpy as np

ANGLE_SENSOR = 11 # degrees

# 0 means light color, 1 means dark color
states = {(0, 0, 0) : 'stop', # special case, depends on prev
          (1, 0, 0) : 'hookL',
          (0, 1, 0) : 'straight',
          (0, 0, 1) : 'hookR',
          (1, 1, 0) : 'turnL',
          (1, 0, 1) : 'straight', # special case, arbitrary value
          (0, 1, 1) : 'turnR',
          (1, 1, 1) : 'straight' # special case, depends on prev
        }

def behavior_lf(drive, lf_sensor, ps_sensor):
    lfFilter = filter.Filter(0.01, -0.4, 0.4) # line follow filter
    interFilter = filter.Filter(0.1, 0.3, 0.8) # intersection filter
    pos = None
    pre = ()
    intersect = False
    end = False
    while True:
        ps_read = ps_sensor.read()
        reading = lf_sensor.read()
        if ps_read[1] <= 0.1:
            return -1
        intersect = intersect_detect(reading, interFilter)
        if intersect == False:
          (pre, pos, end) = line_follow(drive, lfFilter, reading, pre, pos, end)
          if end == True:
              return 0
        else:
            # Align to intersection
            drive.stop()
            time.sleep(0.3)
            drive.go('straight')
            time.sleep(0.4)
            drive.stop()
            time.sleep(0.4)
            return 1


def line_follow(drive, lfFilter, reading, prev, loc, end):
    at_end = end

    # Update filter
    if reading == (0, 0, 1):
        lfFilter.update(0.6)
    elif reading == (0, 1, 1):
        lfFilter.update(0.3)
    elif reading == (1, 0, 0):
        lfFilter.update(-0.6)
    elif reading == (1, 1, 0):
        lfFilter.update(-0.3)
    else:
        pass

    # Determine the action
    # Special case (0, 0, 0)
    if reading == (0, 0, 0):
        if prev == ():
            drive.go('straight')
        elif loc == 'L':
            drive.go('spinR')
        elif loc == 'R':
            drive.go('spinL')
        else:
            drive.go('straight')
            at_end = True

    # Special case (1, 1, 1)
    elif reading == (1, 1, 1) and prev == ():
        drive.go('spinL')

    else:
        drive.go(states[reading])
        prev = reading
    
    # update location
    if lfFilter.tripped_high():
        loc = 'L'
    elif lfFilter.tripped_low():
        loc = 'R'
    else: 
        loc = 'C'
    return (prev, loc, at_end)

def behavior_turn(drive, sensor, turn_dir):
    interFilter = filter.Filter(0.1, 0.3, 0.8) # intersection filter
    dir = turn_dir
    ang = 0
    intersect = True
    while True:
        if intersect == True:
            reading = sensor.read()
            intersect = intersect_detect(reading, interFilter)
            ang = turn(drive, sensor, dir)
            drive.go('stop')
            time.sleep(0.1)
        else:
            return ang   

def turn(drive, sensor, turn_dir):
    t1 = 0
    t2 = 0
    turning = True
    dir = turn_dir
    new_street = False

    ti = time.time() # starting the time for turn

    turnFilter = filter.Filter(0.1, 0.3, 0.8) # turn filter
    slFilter = filter.Filter(0.1, 0.3, 0.8) # left sensor filter
    smFilter = filter.Filter(0.1, 0.3, 0.8) # middle sensor filter    
    srFilter = filter.Filter(0.1, 0.3, 0.8) # right sensor filter

    while turning == True:
        reading = sensor.read()
        
        #Update filter
        if reading[1] == 1:
            turnFilter.update(1)
        else:
            turnFilter.update(0)

        # Update turning
        # must first trip low before it trips high
        if turnFilter.tripped_low():
            new_street = True

        if new_street == True:
            if turnFilter.tripped_high():
                turning = False
            else:
                turning = True

        # Turning Action 
        if turning == True and turn_dir == 'L':
            drive.go('spinL')
        elif turning == True and turn_dir == 'R':
            drive.go('spinR')
        elif turning == False and turn_dir == 'L': 
            dir = 'R'
        elif turning == False and turn_dir == 'R':
            dir = 'L'       
        
        if new_street == True:
            # Update filter
            readL = reading[0]
            readM = reading[1]
            readR = reading[2]
        
            slFilter.update(readL)
            smFilter.update(readM)
            srFilter.update(readR)

            # Update variables
            if turn_dir == 'L':
                if slFilter.tripped_high() and t1 == 0:
                    t1 = time.time()
            else:
                if srFilter.tripped_high() and t1 == 0:
                    t1 = time.time()

            if smFilter.tripped_high() and t2 == 0:
                t2 = time.time()

    tf = time.time() # ending time for turn
    angle = round(calculate_ang(ti, tf, t1, t2) / 45) * 45

    if dir == 'L':
        if (t2 - ti) > 1.5:
            angle = 180
        elif (t2 - ti) <= 1.5 and (t2 - ti) > 0.8:
            angle = 90
        elif (t2 - ti) < 0.8:
            angle = 45
        print(f'Angle Turned: -{angle}')
        # print(f'tm: {t2 - ti}')
    elif dir == 'R':
        if (t2 - ti) > 1.5:
            angle = 180
        elif (t2 - ti) <= 1.5 and (t2 - ti) > 0.8:
            angle = 90
        elif (t2 - ti) < 0.8:
            angle = 45
        print(f'Angle Turned: {angle}')
        # print(f'tm: {t2 - ti}')
    
    return angle

def behavior_turn_report(drive, sensor, turn_dir):
    interFilter = filter.Filter(0.1, 0.3, 0.8) # intersection filter
    dir = turn_dir
    ang = 0
    intersect = True
    ti = 0
    t1 = 0
    t2 = 0

    while True:
        if intersect == True:
            reading = sensor.read()
            intersect = intersect_detect(reading, interFilter)
            (ang, ti, t1, t2) = turn_report(drive, sensor, dir)
        else:
            return (ang, ti, t1, t2)

def turn_report(drive, sensor, turn_dir):
    t1 = 0
    t2 = 0
    turning = True
    dir = turn_dir
    new_street = False

    ti = time.time() # starting the time for turn

    turnFilter = filter.Filter(0.1, 0.3, 0.8) # turn filter
    slFilter = filter.Filter(0.1, 0.3, 0.8) # left sensor filter
    smFilter = filter.Filter(0.1, 0.3, 0.8) # middle sensor filter    
    srFilter = filter.Filter(0.1, 0.3, 0.8) # right sensor filter

    while turning == True:
        reading = sensor.read()
        
        #Update filter
        if reading[1] == 1:
            turnFilter.update(1)
        else:
            turnFilter.update(0)

        # Update turning
        # must first trip low before it trips high
        if turnFilter.tripped_low():
            new_street = True

        if new_street == True:
            if turnFilter.tripped_high():
                turning = False
            else:
                turning = True

        # Turning Action 
        if turning == True and turn_dir == 'L':
            drive.go('spinL')
        elif turning == True and turn_dir == 'R':
            drive.go('spinR')
        elif turning == False and turn_dir == 'L': 
            dir = 'R'
        elif turning == False and turn_dir == 'R':
            dir = 'L'       
        
        if new_street == True:
            # Update filter
            readL = reading[0]
            readM = reading[1]
            readR = reading[2]
        
            slFilter.update(readL)
            smFilter.update(readM)
            srFilter.update(readR)

            # Update variables
            if turn_dir == 'L':
                if slFilter.tripped_high() and t1 == 0:
                    t1 = time.time()
            else:
                if srFilter.tripped_high() and t1 == 0:
                    t1 = time.time()

            if smFilter.tripped_high() and t2 == 0:
                t2 = time.time()

    tf = time.time() # ending time for turn
    angle = round(calculate_ang(ti, tf, t1, t2) / 45) * 45

    if dir == 'L':
        if (t2 - ti) > 1.5:
            angle = 180
        elif (t2 - ti) <= 1.5 and (t2 - ti) > 0.8:
            angle = 90
        elif (t2 - ti) < 0.8:
            angle = 45
        print(f'Angle Turned: -{angle}')
        # print(f'tm: {t2 - ti}')
    elif dir == 'R':
        if (t2 - ti) > 1.5:
            angle = 180
        elif (t2 - ti) <= 1.5 and (t2 - ti) > 0.8:
            angle = 90
        elif (t2 - ti) < 0.8:
            angle = 45
        print(f'Angle Turned: {angle}')
        # print(f'tm: {t2 - ti}')
    
    return (angle, ti, t1, t2)

def report(ti, t1, t2):
    print1 = t1 - ti # t_1 value
    print2 = t2 - ti # t_m value
    print3 = ((t2 - ti) - (t1 - ti)) # t_m - t_1 value
    print4 = (t2 - ti) / ((t2 - ti) - (t1 - ti)) # t_m / (t_m - t_1)
    print(f'{print1},{print2},{print3},{print4}')

def intersect_detect(reading, interFilter):
    if reading == (1, 1, 1):
        
        interFilter.update(1)
    else:
        interFilter.update(0)

    if interFilter.tripped_high():
        return True
    else:
        return False

def calculate_ang(ti, tf, t1, t2):
    sensor_diff = t2 - t1
    ang_vel = ANGLE_SENSOR / sensor_diff
    t_turn = tf - ti
    return t_turn * ang_vel

def check_ahead(sensor):
    avg = [0.5, 0.5, 0.5]
    counter = 0
    while counter < 10:
        reading = sensor.read()
        avg[0] = (avg[0] + reading[0]) / 2
        avg[1] = (avg[1] + reading[1]) / 2
        avg[2] = (avg[2] + reading[2]) / 2
        counter += 1
        
    if avg[0] < 0.3 and avg[1] < 0.3 and avg[2] < 0.3:
        return False
    else:
        return True
    
def check_unblock_ahead(ps_sensor, heading):
    bound = None
    if (heading % 2 == 1):
        bound = 0.65
    else:
        bound = 0.4
    reading = ps_sensor.read()
    # print(f'Ultrasound Reading {reading}')
    # print(f'Ultrasound T/F {reading[0] > bound, reading[1] > bound, reading[2] > bound}')
    return (reading[0] > bound, reading[1] > bound, reading[2] > bound)
    

