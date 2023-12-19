#
#   filter.py
#
#   This includes the class for Filters.
#

class Filter:
    def __init__(self, T, tLow, tHigh):
        self.dt = 0.001
        self.T = T
        self.avg = (tLow  + tHigh) / 2
        self.tLow = tLow
        self.tHigh = tHigh

    def update(self, reading):
        self.avg += (self.dt / self.T) * (reading - self.avg)

    def tripped_low(self):
        return self.avg < self.tLow

    def tripped_high(self):
        return self.avg > self.tHigh

    def get_avg(self):
        return self.avg
    
    def reset(self):
        self.avg = (self.tLow  + self.tHigh) / 2