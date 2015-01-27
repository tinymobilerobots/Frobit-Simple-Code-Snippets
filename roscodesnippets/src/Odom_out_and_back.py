#!/usr/bin/env python
#/****************************************************************************
# Copyright (c) 2014, Ishan Ganeshan <ishan.ganeshan@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

'''
Moves the Robot from Point A to Point B and then back to Point A using the Odometry of the Robot
'''

class OutAndBack():
    def __init__(self):
        
        # Initializes a Node        
        rospy.init_node('odom_out_and_back', anonymous=False)
        
        
        rospy.on_shutdown(self.shutdown)
        
        
        rospy.loginfo("Robot Started")
        
        # Initializes a topic to publish velocity of the robot
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        # Rate at which the velocity would be published
        rate = 20
        

        r = rospy.Rate(rate)
        
        # Intended Linear Speed of the Robot
        linear_speed = 0.2
        
        # Distance of the Goal from the Start Poisition
        goal_distance = 5.0
        
        # Intended Angular Speed of the Robot
        angular_speed = 1.0
        
        # Angle of the Goal
        goal_angle = pi
        
        
        angular_tolerance = radians(2.5)
                
        
        self.tflistener = tf.TransformListener()
        
        
        rospy.sleep(2)
        
        
        self.odom_frame = '/odom'

        # Check whether transform is on /base_link frame or /base_footprint frame
        try:
            self.tflistener.waitForTransform(self.odom_frame, '/base_footprint',rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except(tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tflistener.waitForTransform(self.odom_frame, '/base_link',rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except(tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("No Base Frame Found")
                rospy.signal_shutdown("tf exception")
                
        
        position = Point()
        
        # Loop to move the robot forward and then back again
        for i in range(2):
            
            move_cmd = Twist()
            
            move_cmd.linear.x = linear_speed
            
            (position, rotation) = self.get_odom()
            rospy.loginfo("here")
            x_start = position.x            
            y_start = position.y
            
            distance = 0
            
            # Check whether the robot reached its goal
            while (distance<goal_distance) and not rospy.is_shutdown():
                
                self.cmd_vel.publish(move_cmd)
                
                r.sleep()
                
                (position, rotation) = self.get_odom()
                
                distance = sqrt(pow((position.x - x_start), 2) + 
                                pow((position.y - y_start), 2))
            
            rospy.loginfo("Complete 1st move")  
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
            
            move_cmd.angular.z = angular_speed
            
            last_angle = rotation
            
            turn_angle = 0
            
            # Check whether the robot has turned as much as the goal angle
            while(abs(turn_angle + angular_tolerance)< abs(goal_angle) and 
                  not rospy.is_shutdown()):
                
                self.cmd_vel.publish(move_cmd)
                
                r.sleep()
                
                (position, rotation) = self.get_odom()
                                
                delta_angle = normalize_angle(rotation - last_angle)
                 
                turn_angle += delta_angle
                
                last_angle = rotation
                
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
        
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
    
    '''
    Method which obtains the tf transform of the robot
    '''    
    def get_odom(self):
        try:
            (trans, rot) = self.tflistener.lookupTransform(self.odom_frame,self.base_frame,rospy.Time(0))
        except(tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
    
    '''
    Method to safely shutdown the robot and cancel the goals
    '''
    def shutdown(self):
        
        self.cmd_vel.publish(Twist())
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)

'''
Main method to start the program

'''        
if __name__ == '__main__':
    
    try:
        OutAndBack()
    except:
        rospy.loginfo("Terminated")
         
         
