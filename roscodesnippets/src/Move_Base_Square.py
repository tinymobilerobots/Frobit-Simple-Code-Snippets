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
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

"""
Moves the Robot in a Square  of dimensions 2 metres without Odometry Data

"""

class MoveSquare():
    def __init__(self):
        
        #Initializes a Node 
        rospy.init_node('move_base_square', anonymous=False)
        
        
        rospy.on_shutdown(self.shutdown)
        
        # Initializes a topic to publish the velocity of the robot
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        
        #Defines the size of the square that the robot would traverse
        square_size = rospy.get_param("~square_size", 2.0)
        
        
        quaternions = list()
        

        angles = (pi/2, pi, 3*pi/2, 0)
        
        #Convert the angles from Euler to Quaternions and store it in a list
        for angle in angles:
            
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
            
        
        waypoints = list()
        
        # Add to the list the Coordinates and the Angles that the robot should have at each corner of the square
        waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[0]))
        waypoints.append(Pose(Point(square_size, square_size, 0.0), quaternions[1]))
        waypoints.append(Pose(Point(0.0, square_size, 0.0), quaternions[2]))
        waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
        
        self.init_markers()
        
        
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.marker.points.append(p)
            
        # Initialize an Action Server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        
        rospy.loginfo("Waiting for ActionServer")
        
        # Wait for 60 seconds for the server to make the robot move towards a goal 
        self.move_base.wait_for_server(rospy.Duration(60))
        
        
        rospy.loginfo("Connected to Server")
        
        
        i =0
        
        # Run a loop to move the robot towards the goal 4 times. Once for each corner of the square. 
        while i<4 and not rospy.is_shutdown():
            
            self.markers_pub.publish(self.marker)

            
            goal = MoveBaseGoal()
            
            
            goal.target_pose.pose = waypoints[i]
            
            
            goal.target_pose.header.frame_id = 'map'
            
            
            goal.target_pose.header.stamp = rospy.Time.now()
            
            
            self.move(goal)
            
            i+=1
             
    '''
    Function which sends the goal to the action server
    '''        
    def move(self, goal): 
        
        self.move_base.send_goal(goal)
        
        rospy.loginfo("Robot Started and now moving towards the goal")
        # Wait for 60 seconds for the robot to move towards the goal
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        
        # If the robot not moved towards the goal within 60 seconds then cancel the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Goal Cancelled")
        
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal Succeeded")
         
    '''
    Initialize Markers for displaying on Rviz.
    ''' 
    def init_markers(self):
        
        marker_scale = 0.2
        marker_lifetime = 0
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a':1.0}
        
        self.markers_pub = rospy.Publisher('waypoint_markers', Marker) 
        
        self.marker = Marker()
        
        self.marker.scale.x = marker_scale
        self.marker.scale.y = marker_scale
        self.marker.ns = marker_ns
        self.marker.lifetime = rospy.Duration(marker_lifetime)
        self.marker.id = marker_id
        self.marker.color.r = marker_color['r']
        self.marker.color.g = marker_color['g']
        self.marker.color.b = marker_color['b']
        self.marker.color.a = marker_color['a']
        self.marker.type = Marker.CUBE_LIST
        self.marker.action = Marker.ADD
        
        self.marker.header.frame_id = 'odom'
        self.marker.header.stamp = rospy.Time.now()
        self.marker.points = list()
    
    '''
    Method to safely shutdown the robot and cancel the goals
    '''    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)           
            

'''
Main method to start the Program

'''            
if __name__ == '__main__':
    try:
        MoveSquare()
    except:
        rospy.loginfo("Node Terminated")
    
        
    

    
