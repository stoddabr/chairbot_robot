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

def run():
    port = rospy.get_param('~arduino_port', "/dev/arduino")
    try:
        # ard = serial.Serial(port, 9600, timeout=5)
        ard = serial.Serial("/dev/ttyACM0", 9600, timeout=5)
    except serial.SerialException:
        raise OSError('\n \033[1;32;40m Oops!! looks like the ARDUINO is not connected, try turning it ON or Reconnect \n')
    global send
    print 'Message from arduino: '
    while True:
        try:
            msg = ard.readline()
            # print("The msg from arduino Str=" + msg)
            splits = msg.split("|")
            # print("The full msg from the arduino is stored in this list" , splits)

            if (splits[0] == '0'):
                switchOff = int(splits[0])
                pubOnOffSwitch.publish(switchOff)
                # print("Turn OFF")
            if  (splits[0] == '1'):
                switchOn = int(splits[0])
                pubOnOffSwitch.publish(switchOn)
                # print("Turn ON")
                if (splits[1] == '2'):
                    individualMood = int(splits[1])
                    pubIndividualGroupSwitch.publish(individualMood)
                    print("Individual Mood")
                if (splits[1] == '3'):
                    groupMood = int(splits[1])
                    pubIndividualGroupSwitch.publish(groupMood)
                    print("Group Mood")
                if(splits[2] == '-1' or splits[2] == '11' or splits[2] == '12' or splits[2] == '13'):
                    fsr1 = int(splits[2])
                    pubFsr1.publish(fsr1)
                    # print("fsr1 is:" , fsr1)
                if(splits[3] == '-1' or splits[3] == '21' or splits[3] == '22' or splits[3] == '23'):
                    fsr2 = int(splits[3])
                    pubFsr2.publish(fsr2)
                    # print("fsr2 is:", fsr2)
                if(splits[4] == '-1' or splits[4] == '31' or splits[4] == '32' or splits[4] == '33'):
                    fsr3 = int(splits[4])
                    pubFsr3.publish(fsr3)
                    # print("fsr3 is:", fsr3)
                if(splits[5] == "-1" or splits[5] == "41" or splits[5] == "42" or splits[5] == "43"):
                    fsr4 = int(splits[5])
                    pubFsr4.publish(fsr4)
                    # print("fsr4 is:", fsr4)
                if(splits[6] == "-1" or splits[6] == "51" or splits[6] == "52" or splits[6] == "53"):
                    fsr5 = int(splits[6])
                    pubFsr5.publish(fsr5)
                    # print("fsr5 is:", fsr5)
                if(splits[7] == "-1" or splits[7] == "61" or splits[7] == "62" or splits[7] == "63"):
                    fsr6 = int(splits[7])
                    pubFsr6.publish(fsr6)
                    # print("fsr6 is:", fsr6)

        except IndexError:
            continue
        except KeyboardInterrupt:
            sys.exit()

if __name__ == '__main__':

    rospy.init_node('arduino'+chairbot_number, anonymous=True)
    pubOnOffSwitch = rospy.Publisher('/onOffSwitch'+chairbot_number, Int8, queue_size=10)
    pubIndividualGroupSwitch = rospy.Publisher('/individualGroupSwitch'+chairbot_number, Int8, queue_size=10)
    pubFsr1 = rospy.Publisher('/fsr1'+chairbot_number, Int8, queue_size=10)
    pubFsr2 = rospy.Publisher('/fsr2'+chairbot_number, Int8, queue_size=10)
    pubFsr3 = rospy.Publisher('/fsr3'+chairbot_number, Int8, queue_size=10)
    pubFsr4 = rospy.Publisher('/fsr4'+chairbot_number, Int8, queue_size=10)
    pubFsr5 = rospy.Publisher('/fsr5'+chairbot_number, Int8, queue_size=10)
    pubFsr6 = rospy.Publisher('/fsr6'+chairbot_number, Int8, queue_size=10)
    try:
        run()
    except rospy.ROSInitException:
        pass
