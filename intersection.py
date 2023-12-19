#
#   intersection.py
#
#   This includes the class for Intersection
#
#
import math

class Intersection:
    def __init__(self, coord):
        self.x = coord[0]
        self.y = coord[1]
        self.neighbors = ['UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN']
        self.obstacles = ['UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED'] # unblocked/blocked
        self.cost = math.inf
        self.direction = None
        
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_neighbor(self, heading):
        return self.neighbors[heading]

    def get_neighbors(self):
        return self.neighbors
    
    def is_fully_explored(self, robot, turn_dir):
        for i in range(1, 5):
            if turn_dir == 'L':
                if self.neighbors[(robot.heading + i) % 8] == 'UNKNOWN':
                    return False
            if turn_dir == 'R':
                if self.neighbors[(robot.heading - i) % 8] == 'UNKNOWN':
                    return False
        return True

    def link_neighbor(self, intersect, heading):
        self.neighbors[heading] = intersect

    def link_obstacle(self, heading, type_obstacle):
        self.obstacles[heading] = type_obstacle

    def add_DNE(self, heading):
        if self.neighbors[heading] == "UNKNOWN":
            self.neighbors[heading] = "DNE"
    
    def dne_sweep(self, head, heading, user_dir):
        if head == heading == 0: 
            i = 0
            while i < 7:
                if user_dir == 'L':
                    head = (head + 1) % 8
                    if self.get_neighbor(head) == 'UNKNOWN':
                        self.add_DNE(head)
                    i += 1
                else:
                    head = (head - 1) % 8
                    if self.get_neighbor(head) == 'UNKNOWN':
                        self.add_DNE(head)
                    i += 1
        while head != heading:
            if user_dir == 'L':
                head = (head + 1) % 8
                if head == heading:
                    pass
                else:
                    if self.get_neighbor(head) == 'UNKNOWN':
                        self.add_DNE(head)
            else:
                head = (head - 1) % 8
                if head == heading:
                    pass
                else:
                    if self.get_neighbor(head) == 'UNKNOWN':
                        self.add_DNE(head)

    def add_undriven(self, heading):
        if self.neighbors[heading] == 'UNKNOWN':
            self.neighbors[heading] = 'UNDRIVEN'

    def update_obstacles(self, heading, reading):
        heads = [(heading + 2) % 8, heading, (heading - 2) % 8]
        read = [reading[0], reading[1], reading[2]]
        for i in range(0, len(read)):
            if read[i] == True:
                self.obstacles[heads[i]] = 'UNBLOCKED'
            else:
                self.obstacles[heads[i]] = 'BLOCKED'            
                 
    def get_obstacle(self, heading):
        return self.obstacles[heading]
    
    def get_obstacles(self):
        return self.obstacles

    def clear_obstacles(self):
        self.obstacles = ['UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED', 'UNBLOCKED']

    def set_cost(self, new_cost):
        self.cost = new_cost

    def get_cost(self):
        return self.cost

    def set_direction(self, dir):
        self.direction = dir
    
    def get_direction(self):
        return self.direction
    
    def __repr__(self):
        return f'({self.x}, {self.y})'
    
    def __lt__(self, other):
        return self.cost < other.cost

