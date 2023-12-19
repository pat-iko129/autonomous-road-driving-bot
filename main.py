#!/usr/bin/env python3
#
#   main.py
#   
#   This includes the main actions for goals.
#

# Imports
import pigpio
import sys
import traceback
import drivesystem
import linesensor
import map
import intersection as inter
import behavior
import pickle
import robot

import matplotlib.pyplot as plt
import numpy as np

filename = 'spoiled.pickle'
world = map.Map()
fig = plt.figure()

heading_table = {0 : (0, 1),
                 1 : (-1, 1),
                 2 : (-1, 0),
                 3 : (-1, -1),
                 4 : (0, -1),
                 5 : (1, -1),
                 6 : (1, 0),
                 7 : (1, 1)}

def run(drive, sensor):
    wolfgang = robot.Robot()
    print("Robot ready...")
    
    while True:
        dist = behavior.behavior_lf(drive, sensor)
        wolfgang.set_dist(dist)

        # Marks LineFollow correctly    
        print('SUCCESS')

        # Instantiate first intersection
        if wolfgang.get_pos() == None and wolfgang.get_heading() == None:
            wolfgang.first_intersect(world)
        
        # DNE Encountered
        elif dist == 0:
            curr_intersect = world.get_int(wolfgang.get_pos())
            if curr_intersect.get_neighbor(wolfgang.get_heading()) == 'UNKNOWN':
                curr_intersect.add_DNE(wolfgang.get_heading())
            

        # Instantiate a new intersection
        else:            
            # Did bot move at all?
            if dist > 0:
                wolfgang.set_prev_intersect(world.get_int(wolfgang.get_pos()))

                # Moved to new intersection
                wolfgang.update_pos()

            print(f'Current pos: {wolfgang.get_pos()}')
            print(f'Current heading: {wolfgang.get_heading()}\n')

            # Add new intersection to map or update intersection in map
            world.add_int_to_map(wolfgang.get_pos(), wolfgang.get_heading(), wolfgang.get_prev_intersect())


        # Check ahead for undriven intersection
        if behavior.check_ahead(sensor) == True:
            world.get_int(wolfgang.get_pos()).add_undriven(wolfgang.get_heading())
        elif behavior.check_ahead(sensor) == False:
            world.get_int(wolfgang.get_pos()).add_DNE(wolfgang.get_heading())

        # User input
        user_dir = input('Input desired direction as L, R, S, or EXIT: ').upper()
        while user_dir not in ['L', 'R', 'S', 'EXIT', 'VIEW']:
            user_dir = input('Input desired direction as L, R, S, or EXIT: ').upper()
        
        if user_dir == 'VIEW':
            world.visualize_at_end()
            user_dir = input('Input desired direction as L, R, S, or EXIT: ').upper()
            while user_dir not in ['L', 'R', 'S', 'EXIT']:
                user_dir = input('Input desired direction as L, R, S, or EXIT: ').upper()

        if user_dir in ['L', 'R']:
            wolfgang.set_angle(behavior.behavior_turn(drive, sensor, user_dir))

            wolfgang.update_heading(user_dir)
            
            # DNE Sweep
            world.get_int(wolfgang.get_pos()).dne_sweep(wolfgang.get_prev_heading(), wolfgang.get_heading(), user_dir)
            # Single turns only
            world.single_turns(wolfgang, user_dir)

        elif user_dir == 'S':
            pass # return to line following
        
        elif user_dir == 'EXIT':
            break

        if world.check_complete() == True:
            print('MAP COMPLETE')  

def main():
    # Initialize GPIO and create the objects
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    drive = drivesystem.DriveSystem(io)
    print("Drive ready...")

    sensor = linesensor.LineSensor(io)
    print("LineSensor ready...")

    # Drive in try/except block.
    try:
        run(drive, sensor)
    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shut down cleanly
    print("Turning off...")

    # Saves map !!
    # print(f'Saving the map to {filename}')
    # with open(filename, 'wb') as file:
    #     pickle.dump(world, file)
    
    # print("Printing visual...")
    # world.visualize_at_end()
    
    drive.stop()
    plt.close(fig)
    io.stop()

#    
#   Main
#
if __name__ == "__main__":
    main()
