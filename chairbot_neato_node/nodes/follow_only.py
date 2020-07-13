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


        self.stop_dist = 0.75

        self._pub =rospy.Publisher("/debug", String,queue_size=10)
        self.goal_x = 0
        self.goal_y = 0

        self.center_x = 0
        self.center_y = 0

        # ME #
        self.orient = 0
        self.pose_x = 0
        self.pose_y = 0

        # OTHERS #
        self.posea = PoseStamped().pose.position
        self.poseb = PoseStamped().pose.position
        self.posec = PoseStamped().pose.position

        self.dista = 10
        self.distb = 10
        self.distc = 10
        
        # Formation center
        #### HOW TO CHANGE FIDUICAL GOAL #####
        ### change neato#/pose
        rospy.Subscriber("/neato05/pose", PoseStamped, self.goal_handler, queue_size=10)

        # Neatos
        ############# I AM NEATO 01 ##############################
    
        # double check form of
        if chairbot_number=='01':
            self.i_am = 1
            rospy.Subscriber("/neato01/pose", PoseStamped, self.pose_handler, queue_size=10)
            rospy.Subscriber("/neato02/pose", PoseStamped, self.pose_handlera, queue_size=10)
            rospy.Subscriber("/neato03/pose", PoseStamped, self.pose_handlerb, queue_size=10)
            rospy.Subscriber("/neato04/pose", PoseStamped, self.pose_handlerc, queue_size=10)
        if chairbot_number=='02':
            self.i_am = 2
            rospy.Subscriber("/neato02/pose", PoseStamped, self.pose_handler, queue_size=10)
            rospy.Subscriber("/neato01/pose", PoseStamped, self.pose_handlera, queue_size=10)
            rospy.Subscriber("/neato03/pose", PoseStamped, self.pose_handlerb, queue_size=10)
            rospy.Subscriber("/neato04/pose", PoseStamped, self.pose_handlerc, queue_size=10)
        if chairbot_number=='03':
            self.i_am = 3
            rospy.Subscriber("/neato03/pose", PoseStamped, self.pose_handler, queue_size=10)
            rospy.Subscriber("/neato01/pose", PoseStamped, self.pose_handlera, queue_size=10)
            rospy.Subscriber("/neato02/pose", PoseStamped, self.pose_handlerb, queue_size=10)
            rospy.Subscriber("/neato04/pose", PoseStamped, self.pose_handlerc, queue_size=10)
        if chairbot_number=='04':
            self.i_am = 4
            rospy.Subscriber("/neato04/pose", PoseStamped, self.pose_handler, queue_size=10)
            rospy.Subscriber("/neato01/pose", PoseStamped, self.pose_handlera, queue_size=10)
            rospy.Subscriber("/neato02/pose", PoseStamped, self.pose_handlerb, queue_size=10)
            rospy.Subscriber("/neato03/pose", PoseStamped, self.pose_handlerc, queue_size=10)

             


    def fwd(self):
        SPEED=150
        self._robot.setMotors(-300,-300,SPEED)
        rospy.sleep(0.2)

    def bck(self):
        SPEED=150
        self._robot.setMotors(300,300,SPEED)
        rospy.sleep(0.2)

    def turnRight(self):
        SPEED=200
        self._robot.setMotors(200,-200,SPEED)
        rospy.sleep(0.05)

    def turnLeft(self):
        SPEED=200
        self._robot.setMotors(-200,200,SPEED)
        rospy.sleep(0.05)

    def stop(self):
        SPEED=00
        self._robot.setMotors(00,00,SPEED)
        rospy.sleep(1)
        self._robot.setMotors(00,00,SPEED)

    # Get goal
    def goal_handler(self,data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y



    ######## I AM THIS NEATO #############
    def pose_handler(self,data):
        self.pose_x = data.pose.position.x
        self.pose_y = data.pose.position.y
        qx = data.pose.orientation.x
        qy = data.pose.orientation.y
        qz = data.pose.orientation.z
        qw = data.pose.orientation.w
        self.orient = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])[2]

    #### THESE ARE THE OTHER NEATOES ####
    def pose_handlera(self,data):
        self.posea = data.pose.position
        self.dista = sqrt((self.posea.x-self.pose_x)**2 + (self.posea.y-self.pose_y)**2)

    def pose_handlerb(self,data):
        self.poseb = data.pose.position
        self.distb = sqrt((self.poseb.x-self.pose_x)**2 + (self.poseb.y-self.pose_y)**2)

    def pose_handlerc(self,data):
        self.posec = data.pose.position
        self.distc = sqrt((self.posec.x-self.pose_x)**2 + (self.posec.y-self.pose_y)**2)

    
    def should_stop(self):
        ### WHO GETS PRIORITY ###
        go = 1

        if self.i_am == 1:
            print("I am 1 and I yield to 2, 3, and the Queen")
            if self.dista <= self.stop_dist or self.distb <= self.stop_dist or self.distc <= self.stop_dist:
                go = 0
        elif self.i_am == 2:
            print("I am 2 and I yield to 3 and the Queen")
            if self.distb <= self.stop_dist or self.distc <= self.stop_dist:
                go = 0
        elif self.i_am == 3:
            print("I am 3 and I yield to the Queen")
            if self.distc <= self.stop_dist:
                go = 0
        elif self.i_am == 4:
            print("I am the Queen")
        else:
            print("She doesn't even go here")

        return go

   

    def follow(self):
        goal_angle = atan2(self.goal_y-self.pose_y,self.goal_x-self.pose_x)
        current_angle = self.orient

        ang_error =  goal_angle-current_angle
        pos_error = sqrt((self.goal_x-self.pose_x)*(self.goal_x-self.pose_x) +  (self.goal_y-self.pose_y)*(self.goal_y-self.pose_y))

        ### CHECK IF TOO CLOSE TO OTHER NEATOS ###
        go = self.should_stop()

        ### ONLY MOVE IF NOT STOPPED BY OTHER NEATOS ###
        if go == 1:
            if ang_error > 0:
                if abs(ang_error) > 0.2:
                    self.turnLeft()
                    # self._pub.publish("angle error: " +str(ang_error))
            else:
                if abs(ang_error) > 0.2:
                    self.turnRight()
                    # self._pub.publish("angle error: " +str(ang_error))

            if abs(ang_error) <=0.2:
                if pos_error > self.stop_dist:
                    self.fwd()
                    self._pub.publish("pose01 error: " +str(pos_error))
                else:
                    self.stop()
        else:
            self.stop()

    


    def spin(self):

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
