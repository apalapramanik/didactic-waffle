from cmath import isnan, nan
import rospy
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from testrobots.msg import position
from testrobots.msg import distance
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseAction
from timeit import default_timer as timer
from datetime import timedelta

class move_forward():
    
    def __init__(self):
        
        self.x_cord1 = nan
        self.x_cord2 = nan
        self.x_cord3 = nan
        self.dist1 = nan
        self.dist2 = nan
        self.dist3 = nan

        rospy.Subscriber("position_h1",position,self.position1_callback,queue_size=1)
        rospy.Subscriber("position_h2",position,self.position2_callback,queue_size=1)
        rospy.Subscriber("position_h3",position,self.position3_callback,queue_size=1)
        rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,self.feed_cb,queue_size=1)
        self.distance_human1 = rospy.Publisher("distance_from_human1", distance,queue_size=1)
        self.distance_human2 = rospy.Publisher("distance_from_human2", distance,queue_size=1)
        self.distance_human3 = rospy.Publisher("distance_from_human3", distance,queue_size=1)
        self.goal_sent = False        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server()
    
        
    def position1_callback(self,data): 
        self.x_cord1 = data.x
        self.z_cord1 = data.z
        self.y_cord1 = 0.0
        
    def position2_callback(self,data): 
        self.x_cord2 = data.x
        self.z_cord2 = data.z
        self.y_cord2 = 0.0
        
    def position3_callback(self,data): 
        self.x_cord3 = data.x
        self.z_cord3 = data.z
        self.y_cord3 = 0.0
    
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
        print("result; ", self.result)
        return self.result
    

    def feed_cb(self,feedback):  
        
        global dist
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
            
        if isnan(self.x_cord3) :
            distance_from_human3 = distance()
            print("NO HUMAN 3 ON CAMERA, CAN'T CALCULATE DISTANCE")
            distance_from_human3.distance = 1.26
            self.distance_human3.publish(distance_from_human1)
       
        else: 
            
            distance_from_human3 = distance()
            self.dist3 = math.sqrt((self.x_cord3)**2 + (self.y_cord3)**2 + (self.z_cord3)**2)
            print("dist 3:",self.dist3)
            distance_from_human3.distance = self.dist3
            self.distance_human3.publish(distance_from_human3)
            
            
        if(self.dist1<1) or (self.dist2<1) or (self.dist3<1):
            self.stop()
            rospy.loginfo("CANCEL CURRRENT GOAL")
                
                
    
        
    def stop(self):
        now = rospy.Time.now()
        if self.goal_sent:
            self.move_base.cancel_goal()
            self.goal_sent = False
            self.result = False
            
    
if __name__ == '__main__':
    try:
        
        rospy.init_node('navigation', anonymous=False)
        time_start = rospy.Time.now()
        navigator = move_forward()
       
        orientation = [-0.001809, 0.002350, 3.101696]
        position = {'x': 7.713411, 'y' : 0.519357} 
        quaternion = {'r1' : 0.001, 'r2' : 0.000, 'r3' : -0.481, 'r4' : 0.877} #run command : rosrun tf tf_echo /map /base_link
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        start = timer()
        success = navigator.to_goal(position, quaternion, orientation[0],orientation[1], orientation[2])
        
        again = 0
        
        while(again<1):
            if success:
                rospy.loginfo("Destination reached!")
                again = 1
            else:
                rospy.loginfo("SENDING GOAL AGAIN")
                time.sleep(0.5)
                again = 0
                success = navigator.to_goal(position, quaternion, orientation[0],orientation[1], orientation[2])
      
        
        end = timer()
        print(timedelta(seconds=end-start)) #in seconds
    except rospy.ROSInterruptException:
        rospy.loginfo(" Quitting")
        
            
        
        