#!/usr/bin/env python

import roslib; roslib.load_manifest("chairbot_neato_node")
import rospy, time

from math import sin,cos
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from chairbot_neato_driver.chairbot_neato_driver import Botvac

import socket
import re
global chairbot_number
hostname = socket.gethostname()
chair_id = re.search(r"\d+(\.\d+)?", hostname)
chairbot_number = chair_id.group(0)

class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('teleop'+chairbot_number, anonymous=True)
        self._port = rospy.get_param('~neato_port', "/dev/neato_port")
        rospy.loginfo("Using port: %s"%(self._port))
        self._robot = Botvac(self._port)
        rospy.Subscriber("/joy"+chairbot_number, Joy, self.joy_handler, queue_size=10)
        self._joystick_axes = (-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0)
        self._joystick_buttons = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self._speed = 0
        self._distance = 20
        self._speed_set = 0
        self._speed_ramp = 0
        self._last_x = 0
        self._last_y = 0
        self._x_ramp = 0
        self._y_ramp = 0

    # SQUARE
    def optiona(self):
        SPEED = 125

        self._robot.setMotors(-420,420,SPEED)
        rospy.sleep(3.5)

        self._robot.setMotors(50,50,SPEED)
        rospy.sleep(1)

    # TRIANGLE
    def optionb(self):  
        SPEED=100
        self._robot.setMotors(-100,-100,SPEED)
        rospy.sleep(1)
        self._robot.setMotors(100,100,SPEED)
        rospy.sleep(1)

    # CIRCLE
    def optionc(self):  
        SPEED=200
        self._robot.setMotors(-100,-100,SPEED/2)
        rospy.sleep(1)
        self._robot.setMotors(200,200,SPEED*(1.5))
        rospy.sleep(1)

    # X
    def optiond(self):  
        SPEED=200
        for _ in range(2):
            self._robot.setMotors(-100,-100,SPEED)
            rospy.sleep(1)
            self._robot.setMotors(100,100,SPEED)
            rospy.sleep(1)

    def fwdFast(self):
        SPEED=175
        self._robot.setMotors(-100,-100,SPEED)
        rospy.sleep(1)

    def fwd(self):
        SPEED=300
        self._robot.setMotors(-300,-300,SPEED)
        rospy.sleep(1)

    def back(self):
        SPEED=200
        self._robot.setMotors(200,200,SPEED)
        rospy.sleep(1) 

    def right(self):
        SPEED=150
        self._robot.setMotors(150,-150,SPEED)
        rospy.sleep(1)

    def left(self):
        SPEED=150
        self._robot.setMotors(-150,150,SPEED)
        rospy.sleep(1)

    def turnRight(self):
        SPEED=100
        self._robot.setMotors(220,-220,330)
        rospy.sleep(2.25)

    def turnLeft(self):
        SPEED=100
        self._robot.setMotors(-210,210,SPEED)
        rospy.sleep(2.25)

    def stop(self):
        SPEED=00
        self._robot.setMotors(00,00,SPEED)
        rospy.sleep(1)
        self._robot.setMotors(00,00,SPEED)

    def joy_handler(self, ps):
        self._joystick_buttons =  ps.buttons
        self._joystick_axes = ps.axes

    def spin(self):        
        Lft_t = self._joystick_axes[0]
        Lft_d = self._joystick_axes[1]
        Rgh_t = self._joystick_axes[3]
        Rgh_d = self._joystick_axes[4]
        AageL = self._joystick_axes[2]
        AageR = self._joystick_axes[5]
        L_R = self._joystick_axes[6]
        F_B = self._joystick_axes[7]
        sq = self._joystick_buttons[0]
        xx = self._joystick_buttons[1]
        ci = self._joystick_buttons[2]
        tr = self._joystick_buttons[3]
        self._speed_s = self._joystick_buttons[4]
        self._speed_f = self._joystick_buttons[5]
        AageL_Button = self._joystick_buttons[6]
        AageR_Button = self._joystick_buttons[7]
        share = self._joystick_buttons[8]
        options = self._joystick_buttons[9]
        pressL = self._joystick_buttons[10]
        pressR = self._joystick_buttons[11]
        power = self._joystick_buttons[12]
        self._speed -= ((AageR-1)*10)
        self._speed += ((AageL-1)*10)
        self._speed = int(self._speed)
        if (self._speed<0):
            self._speed=0
        elif (self._speed>330):
            self._speed=330
        
        self._speed_set = self._speed

        ll = (Lft_d*self._distance)
        rr = (Rgh_t*self._distance)
        if (rr>=0):
            x = (-ll - rr)
            y = (-ll + rr)
        else:
            x = (-ll - rr)
            y = (-ll + rr) 

        x=int(x)
        y=int(y)

        speeddif = abs(self._speed_ramp - self._speed_set)

        if (self._speed_ramp<self._speed_set):
            self._speed_ramp += (speeddif/20)
        else:
            self._speed_ramp -= (speeddif/20)

        if (self._speed_ramp<0):
            self._speed_ramp=0
        elif (self._speed_ramp>330):
            self._speed_ramp=330

        if (self._speed_set > 330):
            if (0<x<10):
                x=10
                if (self._speed_ramp>330):
                    self._speed_ramp = 330
            elif (-10<x<0):
                x=-10
                if (self._speed_ramp>330):
                    self._speed_ramp = 330

            if (0<y<10):
                y=10
                if (self._speed_ramp>330):
                    self._speed_ramp = 330
            elif (-10<y<0):
                y=-10
                if (self._speed_ramp>330):
                    self._speed_ramp = 330
        else:
            if (0<x<5):
                x=5
            elif (-5<x<0):
                x=-5

            if (0<y<5):
                y=5
            elif (-5<y<0):
                y=-5

        if (self._x_ramp < self._last_x):
            self._x_ramp += 1
        elif (self._x_ramp == self._last_x):
            pass
        else:
            self._x_ramp -= 1

        if (self._y_ramp < self._last_y):
            self._y_ramp += 1
        elif (self._y_ramp == self._last_y):
            pass
        else:
            self._y_ramp -= 1

        if (x==0 and y==0):
            self._speed_ramp -= (self._speed_set/10)
        else:
            if ((abs(self._x_ramp-x)>20) or (abs(self._y_ramp-y)>20)):
                self._speed_ramp = 50
        
        if (self._speed_ramp<0):
            self._speed_ramp=0

        self._last_x = x
        self._last_y = y
        print (self._x_ramp, x, self._last_x, self._y_ramp, y, self._last_y, self._speed_ramp, self._speed_set)
        self._robot.setMotors(self._x_ramp, self._y_ramp, self._speed_ramp)
        self._robot.flushing()

        if tr == 1:
            self.optiona()
        if ci == 1:
            self.optionb()
        if xx == 1:
            self.optionc()
        if sq == 1:
            self.optiond()

    def shutdown(self):
	    self._robot.setLDS("off")
	    self._robot.setTestMode("off")


if __name__ == "__main__":    
    robot = NeatoNode()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        robot.spin()
        r.sleep()
    # shut down
    robot.shutdown()
