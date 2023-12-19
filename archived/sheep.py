# 
#   sheep.py
# 
#   This class shows herding behavior.
# 

import pigpio
import time
import traceback
import sys
import proximitysensor as ps
import drivesystem as ds
import threading
import ctypes

class Sheep:
    def __init__(self):
        self.active = False
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        
        self.drive = ds.DriveSystem(self.io)
        self.sensor = ps.ProximitySensor(self.io)

    def herd(self):
        while True:
            if self.active == True:

                # Read the latest measurements.
                reading = self.sensor.read()

                uL = reading[0]
                uM = reading[1]
                uR = reading[2]
                
                # Take action
                if uM >= 0.2:
                    if uL < 0.2 and uR < 0.2:
                        self.drive.go('straight')
                    elif uL < 0.2:
                        self.drive.go('turnR')
                    elif uR < 0.2:
                        self.drive.go('turnL')
                    else:
                        self.drive.go('straight')
                elif uM < 0.1:
                    if uL < 0.2 and uR < 0.2:
                        self.drive.go('back')
                    elif uL < 0.2:
                        self.drive.go('backturnL')
                    elif uR < 0.2:
                        self.drive.go('backturnR')
                    else:
                        self.drive.go('back')
                else:
                    if uL < 0.2 and uR < 0.2:
                        self.drive.go('stop')
                    elif uL < 0.2:
                        self.drive.go('backspinL')
                    elif uR < 0.2:
                        self.drive.go('backspinR')
                    else:
                        self.drive.go('stop')
            else:
                self.drive.go('stop')

    def wall_follow_discrete(self, wallSide):
        while True:
            if self.active:   
                # Read the latest measurements.
                reading = self.sensor.read()
                
                # Take action
                d0 = (0.3, 0.3, 0.3)
                d = self.sensor.read()
                e = (d0[0] - d[0], d0[1] - d[1], d0[2] - d[2])

                # assume greater than 0.5 is away from wall
                if e[1] > 0:
                    print(f'stop {e[1]}')
                    self.drive.go('stop')
                else:
                    if wallSide == 'L':
                        if e[0] > -0.1 and e[0] < 0.1: # -0.1 < left < 0.1
                            self.drive.go('straight')
                        elif e[0] >= 0.1 and e[0] < 0.5: # 0.1 <= left < 0.5
                            self.drive.go('turnR')
                        elif e[0] <= -0.1 and e[0] > -0.5: # -0.5 < left <= -0.1
                            self.drive.go('turnL')
                        else:
                            self.drive.go('turnL')
                    else:
                        if e[2] > -0.1 and e[2] < 0.1: # -0.1 < left < 0.1
                            self.drive.go('straight')
                        elif e[2] >= 0.1 and e[2] < 0.5: # 0.1 <= left < 0.5
                            self.drive.go('turnL')
                        elif e[2] <= -0.1 and e[2] > -0.5: # -0.5 < left <= -0.1
                            self.drive.go('turnR')
                        else:
                            self.drive.go('spinR')
            else:
                self.drive.go('stop')

    def wall_follow_prop(self, wallSide):
        while True:
            if self.active:
                # Read the latest measurements.
                reading = self.sensor.read()
                
                # Take action
                d0 = (0.3, 0.3, 0.3)
                d = self.sensor.read()
                e = (d0[0] - d[0], d0[1] - d[1], d0[2] - d[2])

                kL = 80
                kR = 80
                
                if e[1] > 0:
                    self.drive.pwm(0, 0)

                else:
                    if wallSide == 'L':
                        pwmL = 190 + kL * e[0]
                        pwmR = 205 - kR * e[0]

                        if pwmL > 255:
                            pwmL = 255
                        elif pwmL < -255:
                            pwmL = -255
                        if pwmR > 255:
                            pwmR = 255
                        elif pwmR < -255:
                            pwmR = -255
                        self.drive.pwm(pwmL, pwmR)
                    else:
                        pwmL = 190 - kL * e[2]
                        pwmR = 205 + kR * e[2]

                        if pwmL > 255:
                            pwmL = 255
                        elif pwmL < -255:
                            pwmL = -255
                        if pwmR > 255:
                            pwmR = 255
                        elif pwmR < -255:
                            pwmR = -255
                        self.drive.pwm(pwmL, pwmR)
            else:
                self.drive.go('stop')

    def set_active(self, value):
        self.active = value
    
    def run(self):
        try:
            self.herd()
        except BaseException as ex:
            # Report the error, but continue with the normal shutdown.
            print("Ending due to exception: %s" % repr(ex))
            traceback.print_exc()
    
    def shutdown(self):
        self.drive.stop()
        self.sensor.shutdown()
        self.io.stop()