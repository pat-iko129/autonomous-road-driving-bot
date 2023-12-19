#
#   linesensor.py
#
#   This includes a class for a LineSensor object. It instantiates three
#   IR sensors.
#

# Imports
import pigpio
import ir


PIN_IR_LEFT   = 18     # Default GPIO Channel for Left IR Detector
PIN_IR_MIDDLE = 15     # Default GPIO Channel for Middle IR Detector
PIN_IR_RIGHT  = 14     # Default GPIO Channel for Right  IR Detector

class LineSensor:
    def __init__(self, io):
        self.io = io
        self.irL = ir.IR(io, PIN_IR_LEFT)
        self.irM = ir.IR(io, PIN_IR_MIDDLE)
        self.irR = ir.IR(io, PIN_IR_RIGHT)

    def read(self):
        return (self.irL.read(), self.irM.read(), self.irR.read())