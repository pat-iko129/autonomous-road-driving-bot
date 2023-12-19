#
#   drivesystem.py
#
#   This includes a class for a Robot object that can contain multiple
#   components. 
#

# Imports
import pigpio
import sys
import time
import traceback
import motor

# Define the motor pins.
PIN_MOTOR1_LEGA = 7
PIN_MOTOR1_LEGB = 8

PIN_MOTOR2_LEGA = 5
PIN_MOTOR2_LEGB = 6

class DriveSystem:
    def __init__(self, io):
        self.io = io
        self.motor1 = motor.Motor(io, 190, PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB)
        self.motor2 = motor.Motor(io, 205, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB)
        self.action_table = {'straight' : (205, 225, True, True), # left motor hates me :(
                             'veerL' : (175, 215, True, True), 
                             'veerR' : (180, 180, True, True),
                             'steerL' : (155, 215, True, True),
                             'steerR' : (180, 160, True, True),
                             'turnL' : (95, 215, True, True),
                             'turnR' : (180, 90, True, True),
                             'hookL' : (0, 215, True, True),
                             'hookR' : (170, 0, True, True),
                             'spinL' : (180, 190, False, True),
                             'spinR' : (170, 180, True, False),
                             'stop' : (0, 0, True, True),

                             'back' : (205, 225, False, False),
                             'backveerL' : (175, 215, False, False),
                             'backveerR' : (180, 180, False, False),
                             'backsteerL' : (155, 215, False, False),
                             'backsteerR' : (180, 160, False, False),
                             'backturnL' : (95, 215, False, False),
                             'backturnR' : (180, 90, False, False),
                             'backhookL' : (0, 215, False, False),
                             'backhookR' : (170, 0, False, False),
                             'backspinL' : (180, 190, True, False),
                             'backspinR' : (170, 180, False, True),                             
                            }


    def go(self, action):
        if action not in self.action_table.keys():
            raise ValueError('Not a valid action')
        else:
            self.action(self.action_table[action])

    def action(self, settings):
        self.motor1.change_speed(settings[0])
        self.motor2.change_speed(settings[1])
        self.motor1.drive(settings[2])
        self.motor2.drive(settings[3])
    
    def stop(self):
        self.motor1.stop()
        self.motor2.stop()
    
    def pwm(self, pwmL, pwmR):
        forwardL = None
        forwardR = None
        
        if pwmL < 0:
            forwardL = False
        else:
            forwardL = True
        if pwmR < 0:
            forwardR  = False
        else:
            forwardR = True
        
        self.motor1.change_speed(abs(pwmL))
        self.motor2.change_speed(abs(pwmR))
        self.motor1.drive(forwardL)
        self.motor2.drive(forwardR)

def test():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    drive = DriveSystem(io)

    try:
        drive_styles = ['straight', 'veerR', 'veerL', 'steerR', 'steerL',
        'turnR', 'turnL', 'hookR', 'hookL', 'spinR', 'spinL']
        for style in drive_styles:
            print(style)
            drive.go(style)
            time.sleep(4)
            drive.stop()
            input("hit return")
    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shut down cleanly
    print("Turning off...")
    drive.stop()
    io.stop()

def single_test():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    drive = DriveSystem(io)

    try:
        style = input('What driving style would you like: ')
        while style in drive.action_table.keys():
            print(style)
            drive.go(style)
            time.sleep(4)
            drive.stop()
            style = input('What driving style would you like: ')
    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shut down cleanly
    print("Turning off...")
    drive.stop()
    io.stop()

if __name__ == "__main__":
    test()
    # single_test()