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
        # self.x_cord3 = nan
        self.dist1 = nan
        self.dist2 = nan
        # self.result = False
        # self.dist3 = nan
       
       

        
        rospy.Subscriber("position_h1",position,self.position1_callback,queue_size=1)
        rospy.Subscriber("position_h2",position,self.position2_callback,queue_size=1)
        # rospy.Subscriber("position_h3",position,self.position3_callback,queue_size=1)
        rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,self.feed_cb,queue_size=1)
        self.distance_human1 = rospy.Publisher("distance_from_human1", distance,queue_size=1)
        self.distance_human2 = rospy.Publisher("distance_from_human2", distance,queue_size=1)
        # self.distance_human3 = rospy.Publisher("distance_from_human3", distance,queue_size=1)
        self.goal_sent = False        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server()
        
        
   
        
    
        
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
        
    # def position3_callback(self,data): 
    #     self.x_cord3 = data.x
    #     self.z_cord3 = data.z
    #     self.y_cord3 = 0.0
    
    def to_goal(self, pos, quat, or_x,or_y,or_z):
        
        self.goal_sent = True
        self.result = False
        goal1 = MoveBaseGoal()
        goal1.target_pose.header.frame_id = "map"
        goal1.target_pose.header.stamp = rospy.Time.now()
        goal1.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        goal1.target_pose.pose.orientation.x = or_x
        goal1.target_pose.pose.orientation.y = or_y
        goal1.target_pose.pose.orientation.z = or_z
        
        self.move_base.send_goal(goal1,feedback_cb=self.feed_cb)
        
        success = self.move_base.wait_for_result()
        print("success state:", success)

        state = self.move_base.get_state() 

        if success and state == GoalStatus.SUCCEEDED:
            self.result = True
        else:
            self.result = False
           
       
        self.goal_sent = False
        print("result: ", self.result)
        return self.result
    
    def feed_cb(self,feedback):  
            
        if(self.dist1<1.25): #or (self.dist2<1.25): 
            self.stop()                                 #or (self.dist3<1):            
            rospy.loginfo("CANCEL CURRRENT GOAL")
            

        
    def stop(self):
        # now = rospy.Time.now()
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
        
        
        """
        Location 2: 
        
        """
        orientation2 = [-0.000063, 0.003518, -0.107634]
        position2 = {'x': 9.542971, 'y' : -4.489363} 
        quaternion2 = {'r1' : 0.000, 'r2' : 0.001, 'r3' : -0.109, 'r4' : 0.994} #run command : rosrun tf tf_echo /map /base_link
     
        
        """
        Location 3: 
        
        """
        
        orientation3 = [-0.000148, 0.003514, 1.100914]
        position3 = {'x': 15.323753, 'y' : 0.079101} 
        quaternion3 = {'r1' : -0.000, 'r2' : 0.000, 'r3' : 0.508, 'r4' : 0.861} #run command : rosrun tf tf_echo /map /base_link
        
        """
        Location 4: 
        
        """
        
        orientation4 = [-0.000155, 0.003464, -1.043818]
        position4 = {'x': 17.559071, 'y' :-5.762231} 
        quaternion4 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.480, 'r4' : 0.877} #run command : rosrun tf tf_echo /map /base_link
        
        
        """        
        Location 5:
        
        """
        
        orientation5 = [0.000765, 0.003132, 2.965275]
        position5 = {'x': 7.478013, 'y' :-6.061655} 
        quaternion5 = {'r1' : -0.000, 'r2' : 0.000, 'r3' : 0.997, 'r4' : 0.079} #run command : rosrun tf tf_echo /map /base_link
        
        """        
        Location 6 :
        
        """
        
        orientation6 = [-0.000618, 0.002056, 3.040110]
        position6 = {'x': 12.737008, 'y' :-5.354957} 
        quaternion6 = {'r1' : 0.001, 'r2' : -0.001, 'r3' : 0.999, 'r4' : 0.053} #run command : rosrun tf tf_echo /map /base_link
        
        
        
        
        
        
        #---------------------------------------------------------navigate-----------------------------------------------------------------------
        rospy.init_node('navigation', anonymous=False)
        time_start = rospy.Time.now()
        navigator = move_forward()
        # success = False
        rospy.loginfo("Go to (%s, %s) pose", position1['x'], position1['y'])
        start = timer()
        again = 0
        # success = navigator.to_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
        # success = navigator.to_goal(position2, quaternion2, orientation2[0],orientation2[1], orientation2[2])
        # success = navigator.to_goal(position3, quaternion3, orientation3[0],orientation3[1], orientation3[2])
        # success = navigator.to_goal(position4, quaternion4, orientation4[0],orientation4[1], orientation4[2])
        success = navigator.to_goal(position5, quaternion5, orientation5[0],orientation5[1], orientation5[2])
        
        
        
        while(again<1):
            if success:
                rospy.loginfo("Destination reached!")
                again = 1
                
            else:
                rospy.loginfo("SENDING GOAL AGAIN")
                time.sleep(0.5)
                again = 0 
                print(time_start)
                print(rospy.Time.now())
                # success = navigator.to_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
                # success = navigator.to_goal(position2, quaternion2, orientation2[0],orientation2[1], orientation2[2])
                # success = navigator.to_goal(position3, quaternion3, orientation3[0],orientation3[1], orientation3[2])
                # success = navigator.to_goal(position4, quaternion4, orientation4[0],orientation4[1], orientation4[2])
                success = navigator.to_goal(position5, quaternion5, orientation5[0],orientation5[1], orientation5[2])
                # print(time_start)
                # print(time_start-rospy.Time.now())
        
        end = timer()
        seconds = end-start
        # print(timedelta(seconds)) #in seconds
        with open('goal_time.txt', 'a') as file:
            file.write(str(seconds) + '\n')
    except rospy.ROSInterruptException:
        rospy.loginfo(" Quitting")
        
            
        
        