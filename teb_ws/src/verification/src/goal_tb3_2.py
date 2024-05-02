import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion

from cmath import isnan, nan
from std_msgs.msg import Float32
from std_msgs.msg import String
global flag

class TurnRobotNode:
    def __init__(self):
        
       
        rospy.init_node('turn_robot_node_tb3_2')
        self.move_base_client = actionlib.SimpleActionClient('/tb3_2/move_base', MoveBaseAction) 
        self.move_base_client.wait_for_server()
        # if not self.move_base_client.wait_for_server(rospy.Duration(5)):
        #     rospy.logerr("Timeout waiting for move_base1 action server")
        #     rospy.signal_shutdown("Timeout waiting for move_base1 action server")
         
       
        
        rospy.Subscriber("/tb3_2/move_base/feedback",MoveBaseActionFeedback,self.feed_cb,queue_size=10)

        self.goal1 = False
        self.goal2 = False
        self.safe = True
       

      


    def turn_robot(self, angle_degrees):
    
        target_angle = angle_degrees * (math.pi / 180.0)

        turn_goal = MoveBaseGoal()
        turn_goal.target_pose.header.frame_id = 'tb3_2_tf/base_link'
        turn_goal.target_pose.header.stamp = rospy.Time.now()

        turn_goal.target_pose.pose.orientation.z = math.sin(target_angle / 2.0)
        turn_goal.target_pose.pose.orientation.w = math.cos(target_angle / 2.0)
      
        self.move_base_client.send_goal(turn_goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Robot turned by {angle_degrees} degrees successfully.")
            self.goal2 = True
        else:
            rospy.logwarn("Turning the robot failed.")
            self.goal2 = False
            
     
 
    def to_goal(self, pos, quat, or_x,or_y,or_z):
   
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        goal.target_pose.pose.orientation.x = or_x
        goal.target_pose.pose.orientation.y = or_y
        goal.target_pose.pose.orientation.z = or_z
        
        self.move_base_client.send_goal(goal, feedback_cb=self.feed_cb)
        self.move_base_client.wait_for_result()
       

        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("reached location yayyyy  ")
            self.goal1= True
        else:
            rospy.logwarn("couldnt make it :((((  ")
            self.goal1 = False
        print(self.goal1 )
        return self.goal1
    
    
    def feed_cb(self,feedback):             
        
        if self.safe == False:
            self.stop()
            print("cancelled goal")
            self.result1 = False
        else:
            print("keep going")
       
       
           
    def stop(self):
     
        self.move_base_client.cancel_goal()
        self.goal1 = False
    
    
   

if __name__ == '__main__':
    try:
       
       
        orientation = [-0.000155, 0.003464, -1.043818]
        position = {'x': 17.559071, 'y' :-5.762231} 
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.480, 'r4' : 0.877} #run command : rosrun tf tf_echo /map /base_link
      
        
        
        again = 0
        nav_node = TurnRobotNode()
      
        flag = False
        while(again<3):
          
       
            result1 = False
            result1 = nav_node.to_goal(position, quaternion, orientation[0],orientation[1], orientation[2])
            print(result1)
            
            if result1:
                rospy.loginfo("reached destination")
                again = again+1
            else:
            
                rospy.loginfo("changing direction")
                flag = True
                rospy.sleep(1.0)
                result2 = nav_node.turn_robot(45.0)
                print(result2)
            
           
    except rospy.ROSInterruptException:
        pass
