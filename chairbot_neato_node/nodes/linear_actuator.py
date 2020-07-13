#!/usr/bin/python

import roslib
import rospy
import serial
import syslog
import time
import datetime
import calendar
import sys, os
from std_msgs.msg import Int8

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16

import socket
import re
global chairbot_number
hostname = socket.gethostname()
chair_id = re.search(r"\d+(\.\d+)?", hostname)
chairbot_number = chair_id.group(0)

global send
send = 0

def sending(var):
    global send
    send = var.data

def run():
    port = rospy.get_param('~arduino_port', "/dev/arduino")
    try:
        ard = serial.Serial(port, 9600, timeout=5)
    except serial.SerialException:
        raise OSError('\n \033[1;32;40m Oops!! looks like the ARDUINO is not connected, try turning it ON or Reconnect \n')
    global send
    print 'Message from arduino: '
    while True:
	print(send)
        try:
            msg = ard.readline()
	    print(msg)
            if (send==1):
	        # sending in1 is high
                ard.write('i')
	        print("Down")
            elif (send==2):
	        # sending in2 is high
                ard.write('o')
	        print("Up")
            elif (send==3):
                ard.write('s')
                print("stop")
        except IndexError:
            continue
        except KeyboardInterrupt:
            sys.exit()


if __name__ == '__main__':

    rospy.init_node('arduino'+chairbot_number, anonymous=True)
    rospy.Subscriber("/linear_actuator"+chairbot_number, Int8, sending, queue_size=10)
    try:
        run()
    except rospy.ROSInitException:
        pass



			
