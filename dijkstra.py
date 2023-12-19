#!/usr/bin/env python3
#
#   dijkstra.py
#   
#   This includes the implementation of dijkstra's algorithm
#

# Imports
import bisect
import math

import matplotlib.pyplot as plt
import numpy as np
fig = plt.figure()

filename = "map_file.pickle"
world = None

heading_table = {0 : (0, 1),
                 1 : (-1, 1),
                 2 : (-1, 0),
                 3 : (-1, -1),
                 4 : (0, -1),
                 5 : (1, -1),
                 6 : (1, 0),
                 7 : (1, 1)}

# gives back one direction to the goal until the current pos is the goal
def dijkstra_dir(robot):
    curr = robot.map.get_int(robot.get_pos())
    if robot.mode == 'explore':
        goal = robot.map.get_int(robot.d_in_e_goal)
    elif robot.mode == 'goal':
        goal = robot.map.get_int(robot.goal)
    if curr != goal:
        if curr == None:
            robot.set_mode('pause')
            print('I think I am lost, please help >:)')
        else:
            turn_heading = (curr.get_direction() + 4) % 8
            turning = head_diff(robot.get_heading(), turn_heading)
            if turning == 0:
                return 'S'
            elif turning > 0:
                return 'L'
            elif turning < 0:
                return 'R'
    else:
        if robot.mode == 'goal':
            robot.set_goal(None)
            print('REACHED THE GOAL')
            robot.set_mode('pause')
        elif robot.mode == 'explore':
            robot.set_d_in_e_goal(None)
            robot.set_d_in_e(False)
            robot.set_mode('explore')

def dijkstra_unexplore(robot):
    curr = robot.map.get_int(robot.get_pos())
    if robot.mode == 'goal':
        goal = robot.map.get_int(robot.closest)
    if curr != goal:
        if curr == None:
            robot.set_mode('pause')
            print('I think I am lost, please help >:)')
        else:            
            turn_heading = (curr.get_direction() + 4) % 8
            turning = head_diff(robot.get_heading(), turn_heading)
            if turning == 0:
                return 'S'
            elif turning > 0:
                return 'L'
            elif turning < 0:
                return 'R'
    else:
        if robot.mode == 'goal':
            print('REACHED THE GOAL')
            robot.set_mode('pause')

def head_diff(curr_head, goal_head):
    diff = goal_head - curr_head
    if diff < -4:
        diff += 8
    elif diff > 4:
        diff -= 8
    else:
        pass
    return diff

def euclidean(x1, y1, x2, y2):
    return math.sqrt(pow((x2-x1), 2) + pow((y2-y1),2))

def dijkstra(map, goal):
    # reset
    for intersection in map.get_map().values():
        intersection.set_cost(math.inf)
        intersection.set_direction(0)
    
    # initialize 
    map.get_int((goal.get_x(), goal.get_y())).set_cost(0)
    on_deck = [goal]

    seen = []
    # go through on deck queue
    while on_deck: 
        curr = on_deck.pop(0)
        curr_intersect = map.get_int((curr.get_x(), curr.get_y()))
        if curr_intersect not in seen:
            seen.append(curr_intersect)
            heading = 0
            for neighbor in curr_intersect.get_neighbors():
                if neighbor not in ['UNKNOWN', 'DNE', 'UNDRIVEN'] and neighbor not in seen and curr_intersect.get_obstacle(heading) == 'UNBLOCKED':
                    # calculate potential cost
                    potential_cost = curr_intersect.get_cost() + euclidean(curr_intersect.get_x(), curr_intersect.get_y(), 
                                                                           neighbor.get_x(), neighbor.get_y())
                    # calculate potential direction
                    potential_dir = heading
                    
                    # update if new cost is better than old cost
                    if potential_cost < map.get_int((neighbor.get_x(), neighbor.get_y())).get_cost():
                        if map.get_int((neighbor.get_x(), neighbor.get_y())).get_cost() < math.inf:
                            on_deck.remove(neighbor)
                        map.get_int((neighbor.get_x(), neighbor.get_y())).set_cost(potential_cost)
                        map.get_int((neighbor.get_x(), neighbor.get_y())).set_direction(potential_dir)
                        bisect.insort(on_deck, map.get_int((neighbor.get_x(), neighbor.get_y())))
                
                heading = heading + 1 % 8

