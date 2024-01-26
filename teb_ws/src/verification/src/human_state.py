#!/usr/bin/env python

"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot Safety Verification in Construction site
    Advised by: Dr.Dung Hoang Tran    
    
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from monitoring.msg import position


class human_state:
    
    def __init__(self):
        self.human_position_sub = rospy.Subscriber("position_h1", position,self.pose_callback)
        
        
    def pose_callback(self, pose_msg):
        