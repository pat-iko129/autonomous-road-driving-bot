#
#   irdemo.py
#
#   This includes a class for a IR sensor object.
#

# Imports
import pigpio

class IR:
    def __init__(self, io, pin):
        self.io = io
        self.pin = pin

        # Set up the IR pin as input
        io.set_mode(self.pin, pigpio.INPUT)

    def read(self):
        return self.io.read(self.pin)