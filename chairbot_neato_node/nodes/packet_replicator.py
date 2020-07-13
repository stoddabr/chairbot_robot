#!/usr/bin/env python
import roslib; roslib.load_manifest("neato_node");
import rospy

from what_is_my_name import what_is_my_name
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from neato_driver.neato_driver import Botvac


class PacketReplicatorNode:
    '''
    Replicates packets from the  requestMotion and requestStop packets
    '''

    def __init__(self):
        #each chair will have their own topics
        self.chairbot_number = what_is_my_name();
        #topic where we put the replicated packet
        self.chairMovement_topic_name = 'chairMovement' + self.chairbot_number
        #topic where we get the request motion packet
        self.requestMotion_topic_name = 'requestMotion' + self.chairbot_number
        #topic where we get the request stop packet
        self.requestStop_topic_name = 'requestStop' + self.chairbot_number
        self.chairMovementTopic = rospy.Publisher(self.chairMovement_topic_name, Twist, queue_size=30);



        #empty twist packet to replicate which we will fill with the right motion
        self.motion = None
        self.packet = Twist()

        #these are the motions which will be performed when a given CONSTANT is in the packet
        #this is what we actually replicate!
        self.BACKWARD = {
                      'linear': {'x': 150.0, 'y':0.0, 'z':0.0},
                      'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
                   }

        self.FORWARD = {
                    'linear': {'x': -150.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
                 }
        self.LEFT = {
                    'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':50.0}
        }
        self.RIGHT = {
                    'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':-50.0}
        }

        self.STOP_MOTION = {
                  'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                  'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
                }

        #dict mapping the constants to the actual motion dictionaries
        self.MOTIONS = { 'BACKWARD' : self.BACKWARD,
                    "FORWARD": self.FORWARD,
                    'LEFT': self.LEFT,
                    'RIGHT': self.RIGHT,
                    'STOP' : self.STOP_MOTION
                   }

        #this tracks whether we are told to stop or not
        self.STOP_FLAG = False

        #requestMotion topic which receives MOTION commands from the frontends
        rospy.Subscriber(self.requestMotion_topic_name, String, self.motion_callback, queue_size=10)
        #stopMotion topic which receives STOP commands from the frontend
        rospy.Subscriber(self.requestStop_topic_name, String, self.motion_callback, queue_size=10)
        #initialize a ros node
        rospy.init_node('packet_replicator_' + self.chairbot_number)

    def motion_callback(self, msg):
        '''
        takes the motion message request and sets flags OR motion variables based on it
        '''
        print "We got a msg", msg.data

        msg = msg.data #just unrwap the command

        if msg == 'STOP': # we were given the STOP command
            rospy.loginfo("We got a STOP")
            self.STOP_FLAG = True
        else: # we were given a MOTION command
            self.STOP_FLAG = False

        self.motion = self.MOTIONS[msg]
        print("The motion is gonna be ", self.motion)

    def spin(self):
        self.r = rospy.Rate(20) # WHY 20 ???
        while not rospy.is_shutdown():
            if self.motion is None:
                #print "Waiting for motion"
                continue; #try again!

            if self.STOP_FLAG is True:
                rospy.loginfo("Stopping")
            else:
                rospy.loginfo("Moving")
                pass;

            rospy.loginfo("Replicating the packet")
            rospy.loginfo(self.motion)
            #populate the packet with the movememnt commands 
            #for that motion which were set by the motion_callback

            self.packet.linear.x = self.motion['linear']['x']
            self.packet.linear.y = self.motion['linear']['y']
            self.packet.linear.z = self.motion['linear']['z']
            self.packet.angular.x = self.motion['angular']['x']
            self.packet.angular.y = self.motion['angular']['y']
            self.packet.angular.z = self.motion['angular']['z']
            self.chairMovementTopic.publish(self.packet)
            self.r.sleep()

if __name__ == "__main__":
    robot = PacketReplicatorNode()
    robot.spin()
