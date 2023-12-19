#!/usr/bin/env python3
#
#   autonomous.py
#   
#   This populates the map autonomously.
#

# Imports
import pigpio
import sys
import time
import traceback
import drivesystem
import linesensor
import map
import intersection
import behavior
import pickle
import dijkstra
import robot


import numpy as np

heading_table = {0 : (0, 1),
                 1 : (-1, 1),
                 2 : (-1, 0),
                 3 : (-1, -1),
                 4 : (0, -1),
                 5 : (1, -1),
                 6 : (1, 0),
                 7 : (1, 1)}
            
def autonomous(robot):
    if not robot.d_in_e:
        curr_intersect = robot.map.get_int(robot.get_pos())
        first = None
        stepF = None

        steps = [0, 1, -1, 2, -2, 3, -3, 4]

        for step in steps:
            if curr_intersect.neighbors[(robot.get_heading() + step) % 8] == "UNKNOWN":
                if curr_intersect.obstacles[(robot.get_heading() + step) % 8] == "UNBLOCKED":
                    if first == None:
                        first = (robot.get_heading() + step) % 8
                        stepF = step
                        break
        
        for step in steps:
            if curr_intersect.neighbors[(robot.get_heading() + step) % 8] == "UNDRIVEN":
                if curr_intersect.obstacles[(robot.get_heading() + step) % 8] == "UNBLOCKED":
                        if first == None:
                            first = (robot.get_heading() + step) % 8
                            stepF = step
                            break

        if first != None:
            if stepF == 0:
                return 'S'
            elif stepF < 0:
                return 'R'
            elif stepF > 0:
                return 'L'
            
        else: 
            lowest = ()
            incomplete_ints = []
            for intersect in robot.map.intersections.keys():
                for i in range (0, 8):
                    if robot.map.get_int(intersect).neighbors[i] in ['UNKNOWN', 'UNDRIVEN'] and robot.map.get_int(intersect).obstacles[i] == 'UNBLOCKED':
                        print(f'Dijkstra Direct: {i}')
                        print(f'Intersect: {intersect}')
                        print(robot.map.get_int(intersect).get_obstacle(i))

                        dist = dijkstra.euclidean(curr_intersect.get_x(), curr_intersect.get_y(),
                                                  robot.map.get_int(intersect).get_x(), robot.map.get_int(intersect).get_y())
                        incomplete_ints.append((robot.map.get_int(intersect), dist))
                        if lowest == ():
                            lowest = (robot.map.get_int(intersect), dist)
                        elif dist < lowest[1]:
                            lowest = (robot.map.get_int(intersect), dist)

            print(f'DIJKSTRA GOAL: {lowest[0]}')
            dijkstra.dijkstra(robot.map, lowest[0])
            robot.set_d_in_e(True)
            robot.set_d_in_e_goal((lowest[0].get_x(), lowest[0].get_y()))
            return dijkstra.dijkstra_dir(robot)
    else:
        return dijkstra.dijkstra_dir(robot)
