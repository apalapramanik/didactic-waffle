#!/usr/bin/env python
from cmath import isnan, nan
import rospy
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tb_apala.msg import position
from tb_apala.msg import distance
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseAction
from timeit import default_timer as timer
from datetime import timedelta


class move_forward():
    
    def __init__(self):
      
        # self.rob_orientation = None
        # self.rob_position = None
        # self.rob_quaternion1 = None
        # self.rob_quaternion2 = None
        self.current_goal = None
        self.second_goal = None
        self.turned = False
        self.goal_sent = False 
        
        rospy.Subscriber("odom", Odometry , self.rob_pose_callback, queue_size=10)
    
        rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,self.feed_cb,queue_size=10)
        
       
               
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server()
        
        
    def rob_pose_callback(self, msg):
        self.quart = msg.pose.pose.orientation.w
        self.rad = 2*(math.acos(self.quart)) # orientation.w = q[0] = cos(rad/2)   
        print("radian = ", self.rad)      
       
        # x = q[1] = sin(r/2)*x;
        # y = q[2] = sin(r/2)*y;
        # z = q[3] = sin(r/2)*z;
        # w = q[0] = cos(r/2);
        
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        self.orient_x = msg.pose.pose.orientation.x
        self.orient_y = msg.pose.pose.orientation.y
        self.orient_z = msg.pose.pose.orientation.z
        
        # self.rob_orientation = [self.orient_x, self.orient_y, self.orient_z]
        # self.rob_position = {'x': self.pose_x, 'y' :self.pose_y} 
        
      
        self.q1_w = math.cos((math.pi - self.rad)/2)
        # self.q1_x = math.sin((math.pi - self.rad)/2) * self.pose_x
        # self.q1_y = math.sin((math.pi - self.rad)/2) * self.pose_y
        self.q1_z = math.sin((math.pi - self.rad)/2) 
        
        # self.rob_quaternion1 = {'r1' :self.q1_x, 'r2' : self.q1_y, 'r3' : self.q1_z, 'r4' : self.q1_w} 
        
        
        again = 0
        success = self.to_goal(self.q1_w, self.q1_z)
        print(success)

        # while(again<1):
        #     if success:
        #         rospy.loginfo("Destination reached!")
        #         again = 1
                
        #     else:
        #         rospy.loginfo("SENDING GOAL AGAIN")
        #         again = 0            
        #         success = self.to_goal()
    
 
    def to_goal(self, q_z, q_w):
        
        self.goal_sent = True
        self.result = False
        self.current_goal = MoveBaseGoal()
        self.current_goal.target_pose.header.frame_id = "map"
        self.current_goal.target_pose.header.stamp = rospy.Time.now()
     
        self.current_goal.target_pose.pose.orientation.z = q_z  
        self.current_goal.target_pose.pose.orientation.w = q_w        
        
        self.move_base.send_goal(self.current_goal,feedback_cb=self.feed_cb)
        
        success = self.move_base.wait_for_result()
        state = self.move_base.get_state() 
        if success and state == GoalStatus.SUCCEEDED:
            self.result = True
        else:
            self.result = False
        self.goal_sent = False
        print("result: ", self.result)
        return self.result
    
   
        
    
    def feed_cb(self,feedback):  
            
       print("here in feedback")
            
            
        
    def stop(self):
     
        if self.goal_sent:
            self.move_base.cancel_goal()
            self.goal_sent = False
            self.result = False
            
def main():
    rospy.init_node('blah', anonymous=False)         
    navigator = move_forward()   
    # success = False
    # while not navigator.rob_position:  # Wait until rob_position is updated
    #     rospy.sleep(0.1)
    #     print("stuck here")
    # rospy.loginfo("Go to (%s, %s) pose", navigator.rob_position['x'], navigator.rob_position['y'])

    # again = 0
    # success = navigator.to_goal(navigator.rob_position, navigator.rob_orientation, navigator.rob_quaternion1[0],navigator.rob_quaternion1[1], navigator.rob_quaternion1[2] )

    # while(again<1):
    #     if success:
    #         rospy.loginfo("Destination reached!")
    #         again = 1
            
    #     else:
    #         rospy.loginfo("SENDING GOAL AGAIN")
    #         again = 0            
    #         success = navigator.to_goal(navigator.rob_position, navigator.rob_orientation, navigator.rob_quaternion1[0],navigator.rob_quaternion1[1], navigator.rob_quaternion1[2])
    
    while not rospy.is_shutdown():
        rospy.spin()     
            
    
if __name__ == '__main__':
   main()
            
        
        