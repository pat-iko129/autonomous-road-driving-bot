#!/usr/bin/env python3
#
#   robot.py
#   
#   This includes a general robot class
#

# Imports
import intersection as inter
import drivesystem as ds
import proximitysensor as ps
import pigpio
import sys
import linesensor as ls
import autonomous
import behavior
import dijkstra
import map
import traceback
import math


heading_table = {0 : (0, 1),
                 1 : (-1, 1),
                 2 : (-1, 0),
                 3 : (-1, -1),
                 4 : (0, -1),
                 5 : (1, -1),
                 6 : (1, 0),
                 7 : (1, 1)}

class Robot:
    def __init__(self):
        self.start = False
        self.start_coord = None
        self.start_heading = None
        self.stepping = False
        self.mode = None
        self.d_in_e = False
        self.d_in_e_goal = None
        self.goal = None
        self.stepping = False
        self.step = False
        self.map = map.Map()
        self.closest = None
    
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        self.drive = ds.DriveSystem(self.io)
        self.ps_sensor = ps.ProximitySensor(self.io)
        self.lf_sensor = ls.LineSensor(self.io)
        
        self.dist = 0
        self.angle = 0
        self.pos = None
        self.heading = None
        self.prev_heading = None
        self.prev_intersect = None

    def run(self):
        drive = self.drive
        lf_sensor = self.lf_sensor
        world = self.map
        ultrasound = self.ps_sensor

        try:
            while True:
                if self.mode == 'explore' or self.mode == 'goal':
                    if self.stepping == False or (self.stepping == True and self.step == True):
                        self.set_step(False)

                        # Instantiate first intersection with start cmd
                        if self.get_pos() == None and self.get_heading() == None:
                            if self.start == True:
                                self.put_first_intersect(self.start_coord, self.start_heading, world)
                                self.start = False
                                first_intersect = world.get_int(self.get_pos())
                                first_intersect.update_obstacles(self.get_heading(), behavior.check_unblock_ahead(ultrasound, self.get_heading()))
                                world.back_dne(self)
                        
                        if behavior.check_ahead(lf_sensor) == True:
                            if behavior.check_unblock_ahead(ultrasound, self.heading)[1] == True:
                                dist = behavior.behavior_lf(drive,lf_sensor,ultrasound)
                            else:
                                dist = 0 # not moved due to block directly ahead
                        else:
                            dist = 0 # not move due to DNE

                        # Instantiate first intersection
                        if self.get_pos() == None and self.get_heading() == None:
                            self.first_intersect(world)
                            first_intersect = world.get_int(self.get_pos())
                            first_intersect.update_obstacles(self.get_heading(), behavior.check_unblock_ahead(ultrasound, self.get_heading()))
                            world.back_dne(self)
                        
                        # DNE Encountered
                        elif dist == 0:
                            if behavior.check_ahead(lf_sensor) == False: # dne encounter
                                curr_intersect = world.get_int(self.get_pos())
                                if curr_intersect.get_neighbor(self.get_heading()) == 'UNKNOWN':
                                    curr_intersect.add_DNE(self.get_heading())
                            elif behavior.check_unblock_ahead(ultrasound, self.heading)[1] == False: # block encounter
                                curr_intersect = world.get_int(self.get_pos())
                                curr_intersect.update_obstacles(self.get_heading(), behavior.check_unblock_ahead(ultrasound, self.get_heading()))
                            else:
                                print('ERROR: Impossible dist = 0 Case')

                        elif dist == -1:
                            pass

                        # Instantiate a new intersection
                        else:            
                            # Did bot move at all?
                            if dist > 0:
                                self.set_prev_intersect(world.get_int(self.get_pos()))

                                # Moved to new intersection
                                self.update_pos()

                            print(f'Current pos: {self.get_pos()}')
                            print(f'Current heading: {self.get_heading()}\n')

                            # Add new intersection to map or update intersection in map
                            world.add_int_to_map(self.get_pos(), self.get_heading(), self.get_prev_intersect())

                            # Update back DNEs
                            world.back_dne(self)

                        # Print info about intersection 
                        world.inter_info(self.get_pos())

                        if dist == -1: # sudden obstacle
                            print('sudden stop!!')
                            u_turn = behavior.behavior_turn(drive, lf_sensor, 'L')
                            if u_turn != 180:
                                print(f'U-turn angle reported: {u_turn}')
                            u_dist = behavior.behavior_lf(drive, lf_sensor, ultrasound)
                            while u_dist == -1:
                                drive.go('stop')
                                u_dist = behavior.behavior_lf(drive, lf_sensor, ultrasound)
                            if u_dist == 1:
                                self.heading = (self.heading + 4) % 8  
                            pass
                            print(f'Sudden stop current heading: {self.heading}')

                        else :
                            # Check ahead for undriven intersection
                            if behavior.check_ahead(lf_sensor) == True:
                                world.get_int(self.get_pos()).add_undriven(self.get_heading())
                            elif behavior.check_ahead(lf_sensor) == False:
                                world.get_int(self.get_pos()).add_DNE(self.get_heading())

                            # Check unblock ahead for obstacles
                            world.get_int(self.get_pos()).update_obstacles(self.get_heading(), behavior.check_unblock_ahead(ultrasound, self.get_heading()))
                            world.double_obstacles(world.get_int(self.get_pos()), self.get_heading(), world.get_int(self.get_pos()).obstacles[self.get_heading()])
                            
                            turn_dir = self.turning_behavior()
                            
                            self.drive.go('stop')

                            if turn_dir in ['L', 'R']:
                                self.set_angle(behavior.behavior_turn(drive, lf_sensor, turn_dir))
                                self.update_heading(turn_dir)
                                world.get_int(self.get_pos()).update_obstacles(self.get_heading(), behavior.check_unblock_ahead(ultrasound, self.get_heading()))
                                world.double_obstacles(world.get_int(self.get_pos()), self.get_heading(), world.get_int(self.get_pos()).obstacles[self.get_heading()])

                                # DNE Sweep
                                world.get_int(self.get_pos()).dne_sweep(self.get_prev_heading(), self.get_heading(), turn_dir)
                                # Single turns only
                                world.single_turns(self)                                   

                            elif turn_dir == 'S':
                                pass # return to line following

                            if self.mode == 'goal':
                                continue

                            if self.mode == 'explore' and world.check_complete() == True:
                                print('MAP COMPLETE')
                                self.drive.go('stop')
                                self.mode = 'pause'
                        
                            self.drive.go('stop')

                    else:
                        self.drive.go('stop') # finishes lf or turn 
                else:                
                    self.drive.go('stop') # for mode

        except BaseException as ex:
            # Report the error, but continue with the normal shutdown.
            print("Ending Run-Robot due to exception: %s" % repr(ex))
            traceback.print_exc()
            self.drive.go('stop')

    def turning_behavior(self):
        direct = None
        if self.mode == 'explore':
            direct = autonomous.autonomous(self)
            if direct == None: # all blocked
                return direct

            big_angle = self.big_angle(direct)
            if big_angle == (True, False):
                direct = 'R'
            elif big_angle == (False, True):
                direct = 'L'
            return direct
        if self.mode == 'goal':
            # Check block
            print(f'Goal {self.goal}')
            print(f'In map? {self.map.int_exist(self.goal)}')
            if not self.map.int_exist((self.goal)):
                self.closest = self.euclidean_partial()
                print(f'Closest: {self.closest}')
                if self.pos == self.closest:
                    direct = self.best_dir()
                else: 
                    if self.closest == None:
                        print('No closest, clear obstacles')
                        self.mode = 'pause'
                    else:
                        dijkstra.dijkstra(self.map, self.map.get_int(self.closest))
                        direct = dijkstra.dijkstra_unexplore(self)
                        print(f'Not In Map Direct {direct}')
                     
            # Goal is explored
            else:
                dijkstra.dijkstra(self.map, self.map.get_int(self.goal))
                direct = dijkstra.dijkstra_dir(self)
                print(f'In Map Direct {direct}')
            
            big_angle = self.big_angle(direct)
            if big_angle == (True, False):
                direct = 'R'
            elif big_angle == (False, True):
                direct = 'L'
            print(f'Final Direct {direct}')
            return direct
        
    def euclidean_partial(self):
        best_intersect = None
        best_dist = float(math.inf)
        for intersect in self.map.intersections.keys():
            for i in range (0, 8):
                if self.map.get_int(intersect).neighbors[i] in ['UNKNOWN', 'UNDRIVEN'] and self.map.get_int(intersect).obstacles[i] == 'UNBLOCKED':
                    if best_intersect == None:
                        best_intersect = intersect
                        best_dist = dijkstra.euclidean(intersect[0], intersect[1], self.goal[0], self.goal[1])
                    else:
                        dist = dijkstra.euclidean(intersect[0], intersect[1], self.goal[0], self.goal[1])
                        if dist < best_dist:
                            best_intersect = intersect
                            best_dist = dist
        return best_intersect

    def best_dir(self):
        partial_heads = []
        for i in range(0, 8):
            if self.map.get_int(self.closest).neighbors[i] == 'UNKNOWN' and self.map.get_int(self.closest).obstacles[i] == 'UNBLOCKED':
                partial_heads.append(i)
        for i in range(0, 8):
            if self.map.get_int(self.closest).neighbors[i] == 'UNDRIVEN' and self.map.get_int(self.closest).obstacles[i] == 'UNBLOCKED':
                partial_heads.append(i)
        for i in range(0, 8):
            if self.map.get_int(self.closest).neighbors[i] != 'DNE' and self.map.get_int(self.closest).obstacles[i] == 'UNBLOCKED':
                partial_heads.append(i)
        
        possibilities = {}
        for head in partial_heads:
            possibilities[(self.closest[0] + heading_table[head][0], self.closest[1] + heading_table[head][1])] = head
        
        if self.goal in possibilities.keys():
            best_head = possibilities[self.goal]
        else:
            best_pos = list(possibilities.keys())[0]
            best_head = possibilities[best_pos]
            best_dist = dijkstra.euclidean(best_pos[0], best_pos[1], self.closest[0], self.closest[1])
            for pos in possibilities:
                if dijkstra.euclidean(pos[0], pos[1], self.closest[0], self.closest[1]) < best_dist:
                    best_dist = dijkstra.euclidean(pos[0], pos[1], self.closest[0], self.closest[1])
                    best_pos = pos
                    best_head = possibilities[pos]  
    
        turning = dijkstra.head_diff(self.heading, best_head)
        if turning == 0:
            return 'S'
        elif turning > 0:
            return 'L'
        elif turning < 0:
            return 'R'

    def big_angle(self, dir):
        correctL = True
        correctR = True
        curr_int = self.map.get_int(self.pos)
        invalid = ['UNKNOWN', 'DNE']
        for i in range(1, 5):
            if curr_int.neighbors[(self.heading + i) % 8] not in invalid:
                correctL = False
        for i in range(1, 5):
            if curr_int.neighbors[(self.heading - i) % 8] not in invalid:
                correctR = False
        if dir == 'S':
            correctL = False
            correctR = False
        return (correctL, correctR)
    
    def reset(self, full):
        self.mode = 'pause'
        curr_x = int(input('Input current x coordinate: '))
        curr_y = int(input('Input current y coordinate: '))
        curr_heading = int(input('Input current heading: '))
        if not full:
            self.pos = (curr_x, curr_y)
            self.heading = curr_heading
        if full:
            self.map.clear_map()
            self.d_in_e = False
            self.d_in_e_goal = None
            self.goal = None
            self.stepping = False
            self.step = False
            self.closest = None
            self.start = True
            self.dist = 0
            self.angle = 0
            self.pos = None
            self.heading = None
            self.prev_heading = None
            self.prev_intersect = None
            self.start_coord = (curr_x, curr_y)
            self.start_heading = curr_heading
            self.mode = 'explore'

    def shutdown(self):
        self.ps_sensor.shutdown()
        self.drive.stop()
        self.io.stop()

    def set_mode(self, value):
        self.mode = value

    def set_goal(self, coord):
        self.goal = coord

    def set_stepping(self):
        self.stepping = not self.stepping

    def set_step(self, value):
        self.step = value
    
    def set_d_in_e(self, value):
        self.d_in_e = value

    def set_d_in_e_goal(self, value):
        self.d_in_e_goal = value

    def set_dist(self, dist):
        self.dist = dist

    def get_dist(self):
        return self.dist

    def set_angle(self, angle):
        self.angle = angle
    
    def get_angle(self):
        return self.angle

    def update_pos(self):
        new_pos = (self.pos[0] + heading_table[self.heading][0], self.pos[1] + heading_table[self.heading][1])
        self.pos = new_pos
        
    def get_pos(self):
        return self.pos
    
    def set_pos(self, coord):
        self.pos = coord
    
    def set_heading(self, heading):
        self.heading = heading
    
    def update_heading(self, user_dir):
        self.prev_heading = self.heading
        curr = self.map.get_int(self.pos)
        if not curr.is_fully_explored(self, user_dir):
            if user_dir == 'L':
                self.set_heading((self.prev_heading + round(self.angle / 45)) % 8)
            else:
                self.set_heading((self.prev_heading - round(self.angle / 45)) % 8)
        else:        
            print('USING MAP FOR ANGLE')
            for i in range(1, 5):
                if user_dir == 'L':
                    if (curr.neighbors[(self.prev_heading + i) % 8]) != 'DNE':
                        self.set_heading((self.prev_heading + i) % 8)
                        break
                else:
                    if (curr.neighbors[(self.prev_heading - i) % 8]) != 'DNE':
                        self.set_heading((self.prev_heading - i) % 8)
                        break
                

    def get_heading(self):
        return self.heading
    
    def set_prev_heading(self, prev_heading):
        self.prev_heading = prev_heading

    def get_prev_heading(self):
        return self.prev_heading
    
    def set_prev_intersect(self, prev_intersect):
        self.prev_intersect = prev_intersect

    def get_prev_intersect(self):
        return self.prev_intersect

    def first_intersect(self, map):
        self.pos = (0, 0)
        self.heading = 0
        self.prev_heading = 0
        map.add_int(self.pos, inter.Intersection(self.pos))
        self.prev_intersect = map.get_int(self.pos)

    def put_first_intersect(self, pos, heading, map):
        self.pos = pos
        self.heading = heading
        self.prev_heading = heading
        map.add_int(self.pos, inter.Intersection(self.pos))
        self.prev_intersect = map.get_int(self.pos)