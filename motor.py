#
#   motor.py
#
#   This includes a class for a Motor object. It has the capabilities
#   to drive and change the power/speed of the motor. Avoid power less
#   than 100.
#

# Imports
import pigpio

# Defines the constant 
MAX_DUTY_VAL = 255
PWM_FREQ = 1000

# Instantiates a Motor object which has a name, speed, and corresponding pins
# A and B
class Motor:
    def __init__(self, io, default_speed, pinA, pinB):
        self.io = io
        self.speed = default_speed
        self.pinA = pinA
        self.pinB = pinB
        
        # Set up the two pins as output (commanding the motors).
        self.io.set_mode(pinA, pigpio.OUTPUT)
        self.io.set_mode(pinB, pigpio.OUTPUT)

        # Prepare the PWM. Gives the maxiumum value for 100% duty cycle.
        self.io.set_PWM_range(pinA, MAX_DUTY_VAL)
        self.io.set_PWM_range(pinB, MAX_DUTY_VAL)

        # Set the PWM frequency to 1000Hz.
        self.io.set_PWM_frequency(pinA, PWM_FREQ)
        self.io.set_PWM_frequency(pinB, PWM_FREQ)

        # Clear all the pins, just in case.
        self.io.set_PWM_dutycycle(pinA, 0)
        self.io.set_PWM_dutycycle(pinB, 0)

    # Drives one motor and takes in a True/False input for the direction the 
    # motor turns
    def drive(self, dir):
        if dir == True:
            self.io.set_PWM_dutycycle(self.pinA, self.speed)
            self.io.set_PWM_dutycycle(self.pinB, 0)
        
        else:
            self.io.set_PWM_dutycycle(self.pinA, 0)
            self.io.set_PWM_dutycycle(self.pinB, self.speed)
        
    # Changes the power/speed of the motor to a new speed
    def change_speed(self, new_speed):
        self.speed = new_speed

    # Stops the motors
    def stop(self):
        # Clear the PINs (commands).
        self.io.set_PWM_dutycycle(self.pinA, 0)
        self.io.set_PWM_dutycycle(self.pinB, 0)
