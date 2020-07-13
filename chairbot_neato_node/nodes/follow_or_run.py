#!/usr/bin/env python

import roslib; roslib.load_manifest("chairbot_neato_node")
import rospy, time

from math import sin,cos,atan2,sqrt
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16, String
from std_msgs.msg import Int8
from chairbot_neato_driver.chairbot_neato_driver import Botvac
import tf

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
        rospy.Subscriber("/neato05/pose", PoseStamped, self.goal_handler, queue_size=10)
        rospy.Subscriber("/neato01/pose", PoseStamped, self.pose_handler, queue_size=10)
        self._pub =rospy.Publisher("/debug", String,queue_size=10)
        self._goal_x = 0
        self._goal_y = 0
        self._pose_x = 0
        self._pose_y = 0
        self._qx = 0
        self._qy = 0
        self._qz = 0
        self._qw = 0
        self._quaternion = [0,0,0,0]

        self._speed = 0
        self._distance = 20
        self._speed_set = 0
        self._speed_ramp = 0
        self._last_x = 0
        self._last_y = 0
        self._x_ramp = 0
        self._y_ramp = 0


    def fwdFast(self):
        SPEED=175
        self._robot.setMotors(-100,-100,SPEED)
        rospy.sleep(1)

    def fwd(self):
        SPEED=150
        self._robot.setMotors(-300,-300,SPEED)
        rospy.sleep(0.2)

    def back(self):
        SPEED=150
        self._robot.setMotors(200,200,SPEED)
        rospy.sleep(1) 

    def right(self):
        SPEED=150
        self._robot.setMotors(150,-150,SPEED)
        rospy.sleep(1)

    def left(self):
        SPEED=50
        self._robot.setMotors(-150,150,SPEED)
        rospy.sleep(1)

    def turnRight(self):
        SPEED=200
        self._robot.setMotors(220,-220,SPEED)
        rospy.sleep(0.05)

    def turnLeft(self):
        SPEED=200
        self._robot.setMotors(-210,210,SPEED)
        rospy.sleep(0.05)

    def stop(self):
        SPEED=00
        self._robot.setMotors(00,00,SPEED)
        rospy.sleep(1)
        self._robot.setMotors(00,00,SPEED)
        


            # Get goal
    def goal_handler(self,data):
        self._goal_x = data.pose.position.x
        self._goal_y = data.pose.position.y
        

                # Get goal
    def pose_handler(self,data):
        self._pose_x = data.pose.position.x
        self._pose_y = data.pose.position.y
        self._qx = data.pose.orientation.x
        self._qy = data.pose.orientation.y
        self._qz = data.pose.orientation.z
        self._qw = data.pose.orientation.w
        self._quaternion = data.pose.orientation


    def follow(self):
        goal_angle = atan2(self._goal_y-self._pose_y,self._goal_x-self._pose_x)
        current_angle = tf.transformations.euler_from_quaternion([self._qx,self._qy,self._qz,self._qw])[2]
        

        #self._pub.publish("goal: " +str(goal_angle))
        #self._pub.publish("current: " +str(current_angle))
        
        ang_error =  goal_angle-current_angle 
        pos_error = sqrt((self._goal_x-self._pose_x)*(self._goal_x-self._pose_x) +  (self._goal_y-self._pose_y)*(self._goal_y-self._pose_y)) 

     
        if ang_error > 0:
            if abs(ang_error) > 0.2:
                self.turnLeft()                
                self._pub.publish("angle error: " +str(ang_error))
        else:
            if abs(ang_error) > 0.2:
                self.turnRight()
                self._pub.publish("angle error: " +str(ang_error))

        if abs(ang_error) <=0.2:
            if pos_error > 0.8:
                self.fwd()
                self._pub.publish("pose error: " +str(pos_error))
            else: 
                self.stop()

    def run_away(self):
        goal_angle = atan2(self._goal_y-self._pose_y,self._goal_x-self._pose_x)
        current_angle = tf.transformations.euler_from_quaternion([self._qx,self._qy,self._qz,self._qw])[2]
        

        #self._pub.publish("goal: " +str(goal_angle))
        #self._pub.publish("current: " +str(current_angle))
        
        ang_error =  goal_angle-current_angle 
        pos_error = sqrt((self._goal_x-self._pose_x)*(self._goal_x-self._pose_x) +  (self._goal_y-self._pose_y)*(self._goal_y-self._pose_y)) 

     
        if ang_error > 0:
            if abs(ang_error) > 0.3:
                self.turnLeft()                
                self._pub.publish("error: " +str(ang_error))
        else:
            if abs(ang_error) > 0.3:
                self.turnRight()
                self._pub.publish("error: " +str(ang_error))

        if abs(ang_error <=0.3):
            if pos_error < 0.9:
                self.back()

            

    

    def spin(self):        
       
        self._speed = 200
        self._speed = int(self._speed)
        if (self._speed<0):
            self._speed=0
        elif (self._speed>330):
            self._speed=330
        
        self._speed_set = self._speed

        ll = 0
        rr = 0
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

        if (self._speed_set > 150):
            if (0<x<10):
                x=10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150
            elif (-10<x<0):
                x=-10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150

            if (0<y<10):
                y=10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150
            elif (-10<y<0):
                y=-10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150
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
        #print (self._x_ramp, x, self._last_x, self._y_ramp, y, self._last_y, self._speed_ramp, self._speed_set)
        self._robot.setMotors(self._x_ramp, self._y_ramp, self._speed_ramp)
        self._robot.flushing()

    
        self.follow()
        #self.run_away()


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
