#!/usr/bin/env python
from __future__ import division
import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO
from Adafruit_GPIO import I2C
import rospy
from geometry_msgs.msg import Twist
import math

# sudo apt install RPi.GPIO
# pip install adafruit-pca9685
# pip install adafruit-gpio


class SunFounder:
    # STEERING
    # used for the DRIVE_TRAIN_TYPE=SUNFOUNDER_PWM
    STEERING_CHANNEL = 0  # channel on the 9685 pwm board 0-15
    STEERING_LEFT_PWM = 260  # pwm value for full left steering
    STEERING_RIGHT_PWM = 500  # pwm value for full right steering

    # THROTTLE
    # used for the DRIVE_TRAIN_TYPE=SUNFOUNDER_PWM
    THROTTLE_CHANNEL = 4  # channel on the 9685 pwm board 0-15
    THROTTLE_MAX_PWM = 1200  # pwm value for max movement (throttle ->= 1, -1)
    THROTTLE_MIN_PWM = 500  # pwm value for min movement (throttle -> 0)
    # THROTTLE_ZERO_PWM = 0          #pwm value for no movement (throttle = 0)


def map_range(x, X_min, X_max, Y_min, Y_max):
    ''' 
    Linear mapping between two ranges of values 
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range/Y_range

    y = ((x-X_min) / XY_ratio + Y_min)

    return int(y)


class PCA9685:
    ''' 
    PWM motor controller using PCA9685 boards. 
    This is used for most RC Cars
    '''

    def __init__(self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            # replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        # time.sleep(init_delay) # "Tamiya TBLE-02" makes a little leap otherwise

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pulse(pulse)


class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = - math.pi / 4.0
    RIGHT_ANGLE = math.pi / 4.0

    def __init__(self,
                 controller=None,
                 left_pulse=290,
                 right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.pulse = map_range(0, PWMSteering.LEFT_ANGLE, PWMSteering.RIGHT_ANGLE,
                               self.left_pulse, self.right_pulse)
        self.running = True
        rospy.loginfo('PWM Steering created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run(self, angle):
        self.pulse = map_range(angle,
                               self.LEFT_ANGLE, self.RIGHT_ANGLE,
                               self.left_pulse, self.right_pulse)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # set steering straight
        self.run(0)
        self.running = False
        rospy.loginfo('PWM Steering shutdown')


class PWMThrottle:
    """
    SunFounder DC Motor Controller.
    Used for each motor on a differential drive car.
    SunFounder Smart Video Car Kit V2.0/SunFounder PiCar V.
    """
    Motor_A = 17  # GPIO pin for motor a left
    Motor_B = 27  # GPIO pin for motor b right
    PWM_A = 4  # pwm channel for motor a
    PWM_B = 5  # pwm channel for motor b
    FORWARD = False
    BACKWARD = True
    MAX_FORWARD_VELOCITY = 0.22
    MAX_BACKWARD_VELOCITY = -0.22
    T = 0.112 # Track (m)
    R = 0.032 # wheel radius (m)
    L = 0.142 # wheel base (m)

    def __init__(self,
                 max_pulse=1200,
                 min_pulse=500):

        self.max_pulse = 1200 if max_pulse == 0 else max_pulse
        self.min_pulse = 500 if min_pulse == 0 else min_pulse

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWMThrottle.Motor_A, GPIO.OUT)
        GPIO.setup(PWMThrottle.Motor_B, GPIO.OUT)

        self.motor_a = PCA9685(PWMThrottle.PWM_A)
        self.motor_b = PCA9685(PWMThrottle.PWM_B)
        GPIO.output(PWMThrottle.Motor_A, PWMThrottle.FORWARD)
        GPIO.output(PWMThrottle.Motor_B, PWMThrottle.FORWARD)
        self.motor_a.set_pulse(0)
        self.motor_b.set_pulse(0)
        rospy.loginfo('PWM Throttle created')

    def getPWM_throttle(self, throttle, angular, isLeft):
        """
        Calculate the PWM pulse value from throttle, where 1 is full forward and
        -1 is full backwards, 0 is stop. apply differential drive.
        """
        if isLeft:
            v = throttle - angular * PWMThrottle.T / 2.0
        else:
            v = throttle + angular * PWMThrottle.T / 2.0
        
        if v > PWMThrottle.MAX_FORWARD_VELOCITY:
            v = PWMThrottle.MAX_FORWARD_VELOCITY
        elif v < PWMThrottle.MAX_BACKWARD_VELOCITY:
            v = PWMThrottle.MAX_BACKWARD_VELOCITY

        if v == 0:
            direction = PWMThrottle.FORWARD
            pulse = 0
        elif v > 0:
            direction = PWMThrottle.FORWARD
            pulse = int(map_range(v, 0, PWMThrottle.MAX_FORWARD_VELOCITY,
                                  self.min_pulse, self.max_pulse))
        else:
            direction = PWMThrottle.BACKWARD
            pulse = int(map_range(v, PWMThrottle.MAX_BACKWARD_VELOCITY, 0,
                                  self.max_pulse, self.min_pulse))
        return (direction, pulse)

    def run(self, throttle, angular):
        """
        Update the throttle of the motor where 1 is full forward and
        -1 is full backwards.
        """
        dir, pulse = self.getPWM_throttle(throttle, angular, True)
        GPIO.output(PWMThrottle.Motor_A, dir)
        self.motor_a.set_pulse(pulse)
        rospy.loginfo("Left Throttle, v {}, d {}, p {}".format(throttle, dir, pulse))

        dir, pulse = self.getPWM_throttle(throttle, angular, False)
        GPIO.output(PWMThrottle.Motor_B, dir)
        self.motor_b.set_pulse(pulse)
        rospy.loginfo("Right Throttle, v {}, d {}, p {}".format(throttle, dir, pulse))

    def shutdown(self):
        self.motor_a.run(0)
        self.motor_b.run(0)
        GPIO.output(PWMThrottle.Motor_A, GPIO.LOW)
        GPIO.output(PWMThrottle.Motor_B, GPIO.LOW)
        GPIO.cleanup()
        rospy.loginfo('PWM Throttle shutdown')


class TerabotLowLevelCtrl():
    STEERING_SCALE = -1.0

    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        rospy.init_node('terabot_llc')

        self.actuators = {}
        self.actuators['throttle'] = PWMThrottle(
            max_pulse=SunFounder.THROTTLE_MAX_PWM, min_pulse=SunFounder.THROTTLE_MIN_PWM)
        steering_controller = PCA9685(SunFounder.STEERING_CHANNEL)
        self.actuators['steering'] = PWMSteering(controller=steering_controller,
                                                 left_pulse=SunFounder.STEERING_LEFT_PWM,
                                                 right_pulse=SunFounder.STEERING_RIGHT_PWM)

        # --- Create the Subscriber to Twist commands
        self.ros_sub_twist = rospy.Subscriber(
            "mobile_base_controller/cmd_vel", Twist, self.set_actuators_from_cmdvel, queue_size=1)

        # --- Get the last time e got a commands
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 2

    def set_actuators_from_cmdvel(self, message):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        # -- Save the time
        self._last_time_cmd_rcv = time.time()

        # -- Convert vel into servo values
        angluar = message.angular.z * TerabotLowLevelCtrl.STEERING_SCALE
        self.actuators['throttle'].run(message.linear.x, message.angular.z)
        self.actuators['steering'].run(angluar)
        rospy.loginfo("Got a command v = %2.1f  w = %2.1f" %
                      (message.linear.x, message.angular.z))

    def set_actuators_idle(self):
        # -- Convert vel into servo values
        self.actuators['throttle'].run(0, 0)
        self.actuators['steering'].run(0)

    def shutdown(self):
        # -- Convert vel into servo values
        self.actuators['throttle'].shutdown()
        self.actuators['steering'].shutdown()

    @property
    def is_controller_connected(self):
        #rospy.loginfo("is_controller_connected %s" % (time.time() - self._last_time_cmd_rcv))
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    def run(self):

        # --- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #rospy.loginfo("run %s %d" % (self._last_time_cmd_rcv, self.is_controller_connected))
            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()
        self.shutdown()


if __name__ == "__main__":
    llc = TerabotLowLevelCtrl()
    llc.run()
