#!/usr/bin/env python
from cmath import isnan, nan
import rospy
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tb_apala.msg import position
from tb_apala.msg import distance
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseAction
from timeit import default_timer as timer
from datetime import timedelta

class move_forward():
    
    def __init__(self):
        
        self.x_cord1 = nan
        self.x_cord2 = nan
        self.dist1 = nan
        self.dist2 = nan
        self.rob_orientation = None
        self.rob_position = None
        self.rob_quaternion1 = None
        self.rob_quaternion2 = None
        self.current_goal = None
        self.second_goal = None
        self.turned = False
        self.goal_sent = False 
        
        rospy.Subscriber("odom", Pose , self.rob_pose_callback, queue_size=10)
        rospy.Subscriber("position_h1",position,self.position1_callback,queue_size=10)
        rospy.Subscriber("position_h2",position,self.position2_callback,queue_size=10)
        rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,self.feed_cb,queue_size=10)
        
       
        self.distance_human1 = rospy.Publisher("distance_from_human1", distance,queue_size=10)
        self.distance_human2 = rospy.Publisher("distance_from_human2", distance,queue_size=10)
       
               
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server()
        
        
    def rob_pose_callback(self, msg):
        self.quart = msg.pose.pose.orientation.w
        self.rad = 2*(math.acos(self.quart)) # orientation.w = q[0] = cos(rad/2)         
       
        # x = q[1] = sin(r/2)*x;
        # y = q[2] = sin(r/2)*y;
        # z = q[3] = sin(r/2)*z;
        # w = q[0] = cos(r/2);
        
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        self.orient_x = msg.pose.pose.orienttation.x
        self.orient_y = msg.pose.pose.orienttation.y
        self.orient_z = msg.pose.pose.orienttation.z
        
        self.rob_orientation = [self.orient_x, self.orient_y, self.orient_z]
        self.rob_position = {'x': self.pose_x, 'y' :self.pose_y} 
        
        self.rad1 = self.calc_theta(self.pose_x, self.pose_y, self.x_cord1, self.y_cord1)
        self.q1_w = math.cos((math.pi - self.rad1)/2)
        self.q1_x = math.sin((math.pi - self.rad1)/2) * self.x_cord1
        self.q1_y = math.sin((math.pi - self.rad1)/2) * self.y_cord1
        self.q1_z = math.sin((math.pi - self.rad1)/2) * 0.0
        
        self.rob_quaternion1 = {'r1' :self.q1_w, 'r2' : self.q1_x, 'r3' : self.q1_y, 'r4' : self.q1_z} 
        
        
        self.rad2 = self.calc_theta(self.pose_x, self.pose_y, self.x_cord2, self.y_cord2)
        self.q2_w = math.cos((math.pi - self.rad2)/2)
        self.q2_x = math.sin((math.pi - self.rad2)/2) * self.x_cord2
        self.q2_y = math.sin((math.pi - self.rad2)/2) * self.y_cord2
        self.q2_z = math.sin((math.pi - self.rad2)/2) * 0.0
        
        self.rob_quaternion2 = {'r1' :self.q2_w, 'r2' : self.q2_x, 'r3' : self.q2_y, 'r4' : self.q2_z} 
               
    def position1_callback(self,data): 
        self.x_cord1 = data.x
        self.z_cord1 = data.z
        self.y_cord1 = 0.0
        if isnan(self.x_cord1) :
            print("NO HUMAN 1 ON CAMERA, CAN'T CALCULATE DISTANCE")
            distance_from_human1 = distance()
            distance_from_human1.distance = 1.26
            self.distance_human1.publish(distance_from_human1)
        else:
            distance_from_human1 = distance()
            self.dist1 = math.sqrt((self.x_cord1)**2 + (self.y_cord1)**2 + (self.z_cord1)**2)
            print("dist 1:",self.dist1)
            distance_from_human1.distance = self.dist1
            self.distance_human1.publish(distance_from_human1)
        
    def position2_callback(self,data): 
        self.x_cord2 = data.x
        self.z_cord2 = data.z
        self.y_cord2 = 0.0
        
        if isnan(self.x_cord2) :
            distance_from_human2 = distance()
            print("NO HUMAN 2 ON CAMERA, CAN'T CALCULATE DISTANCE")
            distance_from_human2.distance = 1.26
            self.distance_human2.publish(distance_from_human2)
      
        else:            
            distance_from_human2 = distance()
            self.dist2 = math.sqrt((self.x_cord2)**2 + (self.y_cord2)**2 + (self.z_cord2)**2)
            print("dist 2:",self.dist2)
            distance_from_human2.distance = self.dist2
            self.distance_human2.publish(distance_from_human2)

    def to_first_goal(self, pos, quat, or_x,or_y,or_z):
        
        self.goal_sent = True
        self.result = False
        self.current_goal = MoveBaseGoal()
        self.current_goal.target_pose.header.frame_id = "map"
        self.current_goal.target_pose.header.stamp = rospy.Time.now()
        self.current_goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        self.current_goal.target_pose.pose.orientation.x = or_x
        self.current_goal.target_pose.pose.orientation.y = or_y
        self.current_goal.target_pose.pose.orientation.z = or_z        
        
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
    
    def to_second_goal(self, pos, quat, or_x,or_y,or_z):
        
        self.stop()
        self.change_direction = True
        self.result = False
 
        self.second_goal = MoveBaseGoal()
        self.second_goal.target_pose.header.frame_id = "map"
        self.second_goal.target_pose.header.stamp = rospy.Time.now()
        self.second_goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        self.second_goal.target_pose.pose.orientation.x = or_x
        self.second_goal.target_pose.pose.orientation.y = or_y
        self.second_goal.target_pose.pose.orientation.z = or_z        
        
        self.move_base.send_goal(self.second_goal)
        
        success2 = self.move_base.wait_for_result()
        state2 = self.move_base.get_state() 
        if success2 and state2 == GoalStatus.SUCCEEDED:
            self.result = True
        else:
            self.result = False
        print("result: ", self.result)
        return self.result
        
    
    def feed_cb(self,feedback):  
            
        if(self.dist1<1.5): 
            
            #change direction:
            self.stop()
            rospy.loginfo("Goal Changed")
            changed_goal = move_forward()
            self.turned = changed_goal.to_second_goal(self.rob_position, self.rob_orientation, self.rob_quaternion1[0],self.rob_quaternion1[1], self.rob_quaternion1[2] )
            
            
        
    def stop(self):
     
        if self.goal_sent:
            self.move_base.cancel_goal()
            self.goal_sent = False
            self.result = False
            
    
if __name__ == '__main__':
    try:
        
       
        
        
        """
        Location 1:
        
        """
       
        orientation1 = [-0.003284, -0.003284, 2.642751]
        position1 = {'x': 7.972651, 'y' :0.070677} 
        quaternion1 = {'r1' :-0.000, 'r2' : -0.000, 'r3' : 0.960, 'r4' : 0.281} #run command : rosrun tf tf_echo /map /base_link
        
        
        
        
        #---------------------------------------------------------navigate-----------------------------------------------------------------------
        rospy.init_node('navigation', anonymous=False)
        time_start = rospy.Time.now()
        navigator = move_forward()
        # success = False
        rospy.loginfo("Go to (%s, %s) pose", position1['x'], position1['y'])
        start = timer()
        again = 0
        # success = navigator.to_first_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
        success = navigator.to_first_goal(self.rob_position, self.rob_orientation, self.rob_quaternion1[0],self.rob_quaternion1[1], self.rob_quaternion1[2] )

       
        
        
        
        while(again<1):
            if success:
                rospy.loginfo("Destination reached!")
                again = 1
                
            else:
                rospy.loginfo("SENDING GOAL AGAIN")
                # time.sleep(0.5)
                again = 0 
                print(time_start)
                print(rospy.Time.now())
                success = navigator.to_first_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
                
        
        end = timer()
        seconds = end-start
        # print(timedelta(seconds)) #in seconds
        with open('goal_time.txt', 'a') as file:
            file.write(str(seconds) + '\n')
    except rospy.ROSInterruptException:
        rospy.loginfo(" Quitting")
        
            
        
        