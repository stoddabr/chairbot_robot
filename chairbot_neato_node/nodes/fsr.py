#!/usr/bin/env python

import roslib; roslib.load_manifest("chairbot_neato_node")
import rospy, time

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

        rospy.Subscriber('/onOffSwitch'+chairbot_number, Int8, self.onOffswitch_handler, queue_size=10)
        rospy.Subscriber('/individualGroupSwitch'+chairbot_number, Int8, self.individualGroupSwitch_handler, queue_size=10)
        rospy.Subscriber('/groupSwitch', Int8, self.groupSwitch_handler, queue_size=10)
        rospy.Subscriber('/fsr1'+chairbot_number, Int8, self.fsr1_handler, queue_size=10)
        rospy.Subscriber('/fsr2'+chairbot_number, Int8, self.fsr2_handler, queue_size=10)
        rospy.Subscriber('/fsr3'+chairbot_number, Int8, self.fsr3_handler, queue_size=10)
        rospy.Subscriber('/fsr4'+chairbot_number, Int8, self.fsr4_handler, queue_size=10)
        rospy.Subscriber('/fsr5'+chairbot_number, Int8, self.fsr5_handler, queue_size=10)
        rospy.Subscriber('/fsr6'+chairbot_number, Int8, self.fsr6_handler, queue_size=10)
        self._groupMood = rospy.Publisher('/groupSwitch', Int8, queue_size=10)

        self._speed = 0
        self._x=0
        self._y=0
        self._switchStatus = False
        self._individual = False
        self._group = False
        self._fsr1 = 0
        self._fsr2 = 0
        self._fsr3 = 0
        self._fsr4 = 0
        self._fsr5 = 0
        self._fsr6 = 0
        self._Move = False

    def fwd(self):
        self._x = -100
        self._y = -100

    def back(self):
        self._x = 100
        self._y = 100 

    def right(self):
        self._x = 150
        self._y = -150

    def left(self):
        self._x = -150
        self._y = 150
        
    def delay(self):
        rospy.sleep(3)

    def stop(self):
        self._x = 0
        self._y = 0

    def getSpeed (self):
        if((self._fsr1 % 10 == 1)or(self._fsr2 % 10 == 1)or(self._fsr3 % 10 == 1)or(self._fsr4 % 10 == 1)or(self._fsr5 % 10 == 1)or(self._fsr6 % 10 == 1)):
            self._speed = 50
            # print("speed is:", self._speed)
        elif ((self._fsr1 % 10 == 2) or (self._fsr2 % 10 == 2) or (self._fsr3 % 10 == 2) or (self._fsr4 % 10 == 2) or (self._fsr5 % 10 == 2) or (self._fsr6 % 10 == 2)):
            self._speed = 100
            # print("speed is:", self._speed)
        else:
            self._speed = 140
            # print("speed is:", self._speed)
    
    def getrotationSpeed (self):        
        if ((self._fsr1 % 10 == 1) or (self._fsr2 % 10 == 1) or (self._fsr3 % 10 == 1) or (self._fsr4 % 10 == 1)):
            self._speed = 50
            # print("speed is:", self._speed)
        elif ((self._fsr1 % 10 == 2) or (self._fsr2 % 10 == 2) or (self._fsr3 % 10 == 2) or (self._fsr4 % 10 == 2)):
            self._speed = 80
            # print("speed is:", self._speed)
        else:
            self._speed = 110
            # print("speed is:", self._speed)

    def onOffswitch_handler (self, msg):
        if(msg.data == 1):
            self._switchStatus = True
        else:
            self._switchStatus = False
        # print (self._switchStatus)
    
    def individualGroupSwitch_handler(self, msg):
        if (msg.data == 2):
            self._individual = True
            self._group = False
            print (self._individual, self._group)
        else: 
            self._individual = True
            self._group = True
            print (self._individual, self._group)
            self._groupMood.publish(msg.data)
    
    def groupSwitch_handler(self, msg):
        if (msg.data == 3):
            print("the logic is corrert and all should move")
            self._Move = True

    def fsr1_handler(self, msg):
        self._fsr1 = msg.data
        # print(self._fsr1)
    
    def fsr2_handler(self, msg):
        self._fsr2 = msg.data
        # print(self._fsr2)
    
    def fsr3_handler(self, msg):
        self._fsr3 = msg.data
        # print(self._fsr3)
    
    def fsr4_handler(self, msg):
        self._fsr4 = msg.data
        # print(self._fsr4)

    def fsr5_handler(self, msg):
        self._fsr5 = msg.data
        # print(self._fsr5)

    def fsr6_handler(self, msg):
        self._fsr6 = msg.data
        # print(self._fsr6)

    def spin(self):  
        # print(self._x, self._y, self._speed)      
        self._robot.setMotors(self._x, self._y, self._speed)
        self._robot.flushing()

        if (self._Move and self._switchStatus):
            # print("Trun On and be ready to move")
            if ((self._fsr1 != -1) and (self._fsr2 != -1)):
                self.getSpeed()
                self.back()
            elif ((self._fsr3 != -1) and (self._fsr4 != -1)):
                self.getSpeed()
                self.fwd()
            elif (self._fsr5 != -1):
                self.getSpeed()
                self.back()
            elif(self._fsr6 != -1):
                self.getSpeed()
                self.fwd()
            elif (self._fsr1 != -1):
                self.getrotationSpeed()
                self.right()
                # print ("fsr1 is active and trurn right")
            elif (self._fsr4 != -1):
                self.getrotationSpeed()
                self.right()
                # print ("fsr4 is active and trurn right")
            elif (self._fsr2 != -1):
                self.getrotationSpeed()
                self.left()
                # print ("fsr2 is active and trurn left")
            elif (self._fsr3 != -1):
                self.getrotationSpeed()
                self.left()
                # print ("fsr3 is active and trurn left")
            else:
                self._speed = 0
                self.stop()
                # print("Nothing is touched. The robot should stop")
        else:
            self._speed = 0
            self.stop()

    def shutdown(self):
        self._robot.setLDS("off")
        self._robot.setTestMode("off")

if __name__ == "__main__":    
    robot = NeatoNode()
    raspberry = rospy.Rate(20)
    while not rospy.is_shutdown():
        robot.spin()
        raspberry.sleep()
    # shut down
    robot.shutdown()
