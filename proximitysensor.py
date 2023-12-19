#
#   proximitysensor.py
#   
#   This includes the class for Ultrasound and ProximitySensor.
#

import pigpio
import time
import traceback
import sys
import threading

SPEED_OF_SOUND = 343 / 2 # in meters per second
BOT_SPEED_OF_SOUND = 172.414 # in meters per second

PIN_LEFT_TRIG = 13
PIN_MIDDLE_TRIG = 19
PIN_RIGHT_TRIG = 26

PIN_LEFT_ECHO = 16
PIN_MIDDLE_ECHO = 20
PIN_RIGHT_ECHO = 21

class Ultrasound:
    def __init__(self, io, pintrig, pinecho):
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho

        # Saved information
        self.risetick = 0
        self.tof = 0
        self.dist = 0

        # Set up the two pins as output/input
        io.set_mode(pintrig, pigpio.OUTPUT)
        io.set_mode(pinecho, pigpio.INPUT)
    
        # Set up the callbacks
        cbrise = io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        cbfall = io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)
    
    def trigger(self):
        self.io.write(self.pintrig, 1)
        time.sleep(0.000010)
        self.io.write(self.pintrig, 0)

    # level is 1 for rising edge
    # ticks are the time the edge happened in us
    def rising(self, pin, level, ticks):
        # Save the ticks (for the later distance computation)
        self.risetick = ticks
        
    def falling(self, pin, level, ticks):
        # Use the ticks to compute time of flight and distance
        falltick = ticks
        deltatick = falltick - self.risetick
        if (deltatick < 0):
            deltatick += 2 ** 32
        self.tof = deltatick # in microseconds
        self.dist = deltatick / 1000000 * BOT_SPEED_OF_SOUND # in meters

    def read(self):
        return self.dist
    
    def get_tof(self):
        return self.tof

class ProximitySensor:
    def __init__(self, io):
        self.io = io
        self.uL = Ultrasound(io, PIN_LEFT_TRIG, PIN_LEFT_ECHO)
        self.uM = Ultrasound(io, PIN_MIDDLE_TRIG, PIN_MIDDLE_ECHO)
        self.uR = Ultrasound(io, PIN_RIGHT_TRIG, PIN_RIGHT_ECHO)
       
        print("Starting triggering thread...")
        self.triggering = True
        self.thread = threading.Thread(name="TriggerThread", target=self.run)
        self.thread.start()
        time.sleep(0.1) # Wait for the first measurements to arrive

    def read(self):
        return (self.uL.read(), self.uM.read(), self.uR.read())

    def read_tofs(self):
        return(self.uL.get_tof(), self.uM.get_tof(), self.uR.get_tof())
    
    def trigger(self):
        self.uL.trigger()
        self.uM.trigger()
        self.uR.trigger()
    
    def run(self):
        while self.triggering:
            self.trigger()
            time.sleep(0.05)
    
    def shutdown(self):
        self.triggering = False
        print("Waiting for triggering thread to finish...")
        self.thread.join()
        print("Triggering thread returned.")


def run(sensor): # from demo
    while True:
        # sensor.trigger()
        time.sleep(0.100)

        # Read/report
        distances = sensor.read()
        tofs = sensor.read_tofs()
        print("Distances = (%6.3fm, %6.3fm, %6.3fm)" % distances)
        print("TOF = (%6.3fus, %6.3fus, %6.3fus)" % tofs)

def main():
    # Initialize GPIO and create the objects
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    
    sensor = ProximitySensor(io)
    # Run in try/except block.
    try:
        run(sensor) # from demo
    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    
    sensor.shutdown()
    io.stop()
    

#    
#   Main
#
if __name__ == "__main__":
    main()