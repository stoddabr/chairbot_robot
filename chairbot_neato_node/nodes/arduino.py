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
send = False

def sending(var):
    global send
    if (var.data == 1):
        send = True
    elif (var.data == 0):
        send = False

def run():
    port = rospy.get_param('~arduino_port', "/dev/arduino")
    try:
        ard = serial.Serial(port, 9600, timeout=5)
    except serial.SerialException:
        raise OSError('\n \033[1;32;40m Oops!! looks like the ARDUINO is not connected, try turning it ON or Reconnect \n')
    global send
    print 'Message from arduino: '
    while True:
        try:
            msg = ard.readline()
            splits = msg.split(' ')
            if (splits[1] == '1'):
            	touch = int(splits[0])
            	pub_touch.publish(touch)
            	print(touch)
            elif (splits[1] == '2'):
            	release = int(splits[0]) + 12
            	pub_touch.publish(release)
            	print(release)
            if (send):
                ard.write('i') 
            else:
                ard.write('o')
        except IndexError:
            continue
        except KeyboardInterrupt:
            sys.exit()

if __name__ == '__main__':

    rospy.init_node('arduino'+chairbot_number, anonymous=True)
    pub_touch = rospy.Publisher('/touches'+chairbot_number, Int8, queue_size=10)
    rospy.Subscriber("/led"+chairbot_number, Int8, sending, queue_size=10)
    try:
        run()
    except rospy.ROSInitException:
        pass


			
