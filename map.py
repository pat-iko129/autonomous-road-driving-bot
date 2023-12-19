#
#   map.py
# 
#   This includes the class for Map
#
import intersection as inter
import matplotlib.pyplot as plt
import copy

heading_table = {0 : (0, 1),
                 1 : (-1, 1),
                 2 : (-1, 0),
                 3 : (-1, -1),
                 4 : (0, -1),
                 5 : (1, -1),
                 6 : (1, 0),
                 7 : (1, 1)} 

class Map:
    def __init__(self):
        self.intersections = {}

    def int_exist(self, coord):
        if coord in self.intersections.keys():
            return True
        else:
            return False

    def get_int(self, coord):
        if self.int_exist(coord):
            return self.intersections[coord]
        else:
            return None
    
    def add_int(self, coord, intersection):
        self.intersections[coord] = intersection

    def add_int_to_map(self, position, heading, prev_intersect): # add intersection to map
        pos = position
        head = heading
        prev_int = prev_intersect
        
        if self.int_exist(pos) == False: # intersection does not already exist
            curr_int = inter.Intersection(pos)
            self.add_int(pos, curr_int)
        else:
            curr_int = self.get_int(pos)
        
        prev_int.link_neighbor(curr_int, (head % 8)) # updates the one we are moved off of
        (curr_int).link_neighbor(prev_int, (head + 4) % 8) # updates the one we curr on

    def get_map(self):
        return self.intersections
    
    def map_info(self):
        for intersect in self.get_map().values():
            print(f'Intersection: {(intersect.x, intersect.y)}')
            print(f'Neighbors: {intersect.neighbors}')
            print(f'Obstacles: {intersect.obstacles}')
            print(f'Cost: {intersect.cost}')
            print(f'Direction: {intersect.direction}')
    
    def inter_info(self, coord):
        if self.int_exist(coord):
            intersect = self.intersections[coord]
            print(f'Intersection: {(intersect.x, intersect.y)}')
            print(f'Neighbors: {intersect.neighbors}')
            print(f'Obstacles: {intersect.obstacles}')
            print(f'Cost: {intersect.cost}')
            print(f'Direction: {intersect.direction}')
    
        else:
            print("Intersect Does Not Exist")

    def check_complete(self):
        world = self.get_map()
        for pos in world.keys():
            i = 0
            for neighbor in world[pos].get_neighbors():
                if neighbor == 'UNKNOWN':
                    if world[pos].obstacles[i] == 'UNBLOCKED':
                        return False
                elif neighbor =='UNDRIVEN':
                    if world[pos].obstacles[i] == 'UNBLOCKED':
                        return False
                i += 1
        return True
    
    def visualize_simultaneous(self):
        world = self.get_map()
        for point in world.keys():
            intersection = world[point]
            surrounding_pts = []
            for i in range(0, 8):
                surrounding_pts.append((intersection.x + heading_table[i][0], intersection.y + heading_table[i][1]))       
            j = 0
            for neighbor in intersection.neighbors:
                line_type = ''
                color = 'k'
                if neighbor == 'UNKNOWN':
                    color = 'r'
                elif neighbor == 'UNDRIVEN':
                    line_type = '--'
                elif neighbor == 'DNE':
                    line_type = ':'
                else:
                    line_type = '-'                

                xpoints = [intersection.x, surrounding_pts[j][0]]
                ypoints = [intersection.y, surrounding_pts[j][1]]
                plt.plot(xpoints, ypoints, color = color, linestyle = line_type)
                j += 1
        plt.pause(0.001)
        plt.clf()
            
    def visualize_at_end(self, curr_pos):
        world = self.get_map()
        for point in world.keys():
            intersection = world[point]
            surrounding_pts = []
            for i in range(0, 8):
                surrounding_pts.append((intersection.x + heading_table[i][0], intersection.y + heading_table[i][1]))       
            j = 0
            
            for neighbor in intersection.neighbors:
                line_type = ''
                color = 'k'
                if neighbor == 'UNKNOWN':
                    color = 'r'
                elif neighbor == 'UNDRIVEN':
                    line_type = '--'
                elif neighbor == 'DNE':
                    line_type = ':'
                else:
                    line_type = '-'                

                xpoints = [intersection.x, surrounding_pts[j][0]]
                ypoints = [intersection.y, surrounding_pts[j][1]]
                plt.plot(xpoints, ypoints, color=color, linestyle=line_type)
                j += 1
        plt.plot(curr_pos[0], curr_pos[1], color='r', marker='.', markersize='12')
        plt.show()
    
    def visualize_overlay(self):
        # visualize map
        world = self.get_map()
        overlay = {}
        for inte in world.values():
            overlay[inte] = inte.direction

        for point in world.keys():
            intersection = world[point]
            surrounding_pts = []
            for i in range(0, 8):
                surrounding_pts.append((intersection.x + heading_table[i][0], intersection.y + heading_table[i][1]))       
            j = 0
            for neighbor in intersection.neighbors:
                line_type = ''
                color = 'k'
                if neighbor == 'UNKNOWN':
                    color = 'r'
                elif neighbor == 'UNDRIVEN':
                    line_type = '--'
                elif neighbor == 'DNE':
                    line_type = ':'
                else:
                    line_type = '-'                

                xpoints = [intersection.x, surrounding_pts[j][0]]
                ypoints = [intersection.y, surrounding_pts[j][1]]
                plt.plot(xpoints, ypoints, color = color, linestyle = line_type)
                j += 1
        
        # overlay
        surrounding_pts = []
        j = 0
        for intersect in overlay.keys():
            color_head = (overlay[intersect] + 4) % 8
            surrounding_pts.append((intersect.get_x() + heading_table[color_head][0], intersect.get_y() + heading_table[color_head][1]))
            xpoints = [intersect.x, surrounding_pts[j][0]]
            ypoints = [intersect.y, surrounding_pts[j][1]]
            plt.plot(xpoints, ypoints, color = 'r', linestyle = '-')
            j += 1
        plt.pause(0.001)

        
    def single_turns(self, robot):
        self.get_int(robot.get_pos()).add_DNE((robot.get_heading() + 1) % 8)
        self.get_int(robot.get_pos()).add_DNE((robot.get_heading() - 1) % 8)

    def back_dne(self, robot):
        self.get_int(robot.get_pos()).add_DNE((robot.get_heading() + 3) % 8)
        self.get_int(robot.get_pos()).add_DNE((robot.get_heading() - 3) % 8)

    def clear_map(self):
        self.intersections.clear()
    
    def make_copy(self):
        new_Map = copy.deepcopy(self)
        return new_Map

    def double_obstacles(self, curr_int, heading, type_obstacle):
        next_int = (curr_int.x + heading_table[heading][0], curr_int.y + heading_table[heading][1])
        if self.get_int(next_int) != None:
            self.get_int(next_int).link_obstacle((heading + 4) % 8, type_obstacle)
