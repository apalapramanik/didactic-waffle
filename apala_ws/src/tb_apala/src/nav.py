import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion
from tb_apala.msg import position
from tb_apala.msg import distance
from tb_apala.msg import ttc
from cmath import isnan, nan
from std_msgs.msg import Float32
global flag 

class TurnRobotNode:
    def __init__(self):
        rospy.init_node('turn_robot_node')
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)        
        self.move_base_client.wait_for_server()

        self.goal1 = False
        self.goal2 = False
        self.current_orientation = 0.0
        self.dist1 = nan
        self.path_poses = []
        self.next_200_poses = []
        self.min_ttc_to_turn = 0.8
        self.min_ttc_to_stop = 0.8
        self.speed_h1 = 0.0
        self.prev_point = None
        self.prev_time = None
        
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,self.feed_cb,queue_size=10)
        rospy.Subscriber("position_h1",position,self.position1_callback,queue_size=10)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        
        self.distance_human1 = rospy.Publisher("distance_from_human1", distance,queue_size=1)
        self.speed_human1 = rospy.Publisher('/human1_speed', Float32, queue_size=10)
        self.ttc_h1 = rospy.Publisher('/ttc_human1', Float32, queue_size=10)
        self.odom_rad = rospy.Publisher('/rob_rad', Float32,queue_size=10 )
        self.ttc_threshold =  rospy.Publisher('/min_ttc', Float32,queue_size=10)
    
    
    def position1_callback(self,data): 
        # print(data)
          
            
        if self.prev_point is not None and self.prev_time is not None:
            current_time = rospy.Time.now()
            time_difference = (current_time - self.prev_time).to_sec()
            print(time_difference)

            distance_h1 = self.calculate_distance(self.prev_point, data)
            print("distance: ", distance_h1)
            if time_difference>0:
                self.speed_h1 = distance_h1 / time_difference
                print("speed: ", self.speed_h1)

                self.speed_human1.publish(self.speed_h1)
         

            self.prev_time = current_time
            self.prev_point = data
        else:
            self.prev_time = rospy.Time.now()
            self.prev_point = data
            
        self.x_cord1 = data.x
        self.z_cord1 = data.z
        self.y_cord1 = 0.0   
            
        if isnan(self.x_cord1) :
            print("NO HUMAN 1 ON CAMERA, CAN'T CALCULATE DISTANCE")
            distance_from_human1 = distance()
            distance_from_human1.distance = 1.9
            self.distance_human1.publish(distance_from_human1)
        else:
            distance_from_human1 = distance()
            self.dist1 = math.sqrt((self.x_cord1)**2 + (self.y_cord1)**2 + (self.z_cord1)**2)
            print("dist 1:",self.dist1)
            distance_from_human1.distance = self.dist1
            self.distance_human1.publish(distance_from_human1)
  
            
    def time_to_collision(self, dist,  speed):        
        ttc = dist / (speed+0.25)
        return ttc
    
   
    def calculate_distance(self, point1, point2):
        dx = point2.x - point1.x
        dy = point2.z - point1.z
        distance = math.sqrt(dx**2 + dy**2)
        return distance


    def odom_callback(self, odom_msg):
        orientation_quaternion = odom_msg.pose.pose.orientation
        self.current_orientation = math.atan2(2.0 * (orientation_quaternion.w * orientation_quaternion.z + orientation_quaternion.x * orientation_quaternion.y),
                                              1.0 - 2.0 * (orientation_quaternion.y * orientation_quaternion.y + orientation_quaternion.z * orientation_quaternion.z))
        # with open('orientation.txt', 'a') as file:
        #     file.write(str(self.current_orientation) + '\n')
        self.rad_msg = Float32()
        self.rad_msg.data = self.current_orientation
        self.odom_rad.publish(self.rad_msg)
        
  
        
    def path_callback(self, path_msg):
       
        path_poses = path_msg.poses
        # print("length =", len(path_poses))
        
        self.next_200_poses = []
        
        # Store the next 200 poses (or fewer if the plan is shorter)
        for pose in path_poses[:200]:
            self.next_200_poses.append(pose)


    def turn_robot(self, angle_degrees):
        # target_angle = self.current_orientation + angle_degrees * (math.pi / 180.0)
        target_angle = angle_degrees * (math.pi / 180.0)

        turn_goal = MoveBaseGoal()
        turn_goal.target_pose.header.frame_id = 'base_link'
        turn_goal.target_pose.header.stamp = rospy.Time.now()

        turn_goal.target_pose.pose.orientation.z = math.sin(target_angle / 2.0)
        turn_goal.target_pose.pose.orientation.w = math.cos(target_angle / 2.0)
        # turn_goal.target_pose.pose.position.x = 0.5
        
    
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
            
        
        
        self.ttc_1_msg = Float32()
        self.ttc_1_msg.data = self.time_to_collision(self.dist1, self.speed_h1)        
        self.ttc_h1.publish(self.ttc_1_msg)
        print("TTC: ", self.ttc_1_msg.data)
        if self.ttc_1_msg.data < self.min_ttc_to_stop and flag == False:
            self.stop()
            print("cancelled goal")
            self.result1 = False
        # with open('ttc.txt', 'a') as file:
        #     file.write(str(self.ttc_1) + '\n')
        self.ttc_threshold_msg =  Float32()
        self.ttc_threshold_msg.data = self.min_ttc_to_stop
        self.ttc_threshold.publish(self.ttc_threshold_msg)
      
           
    def stop(self):
     
        self.move_base_client.cancel_goal()
        self.goal1 = False
    
    
   

if __name__ == '__main__':
    try:
        """
        Location 1:
        
        """
       
        orientation1 = [-0.003284, -0.003284, 2.642751]
        position1 = {'x': 7.972651, 'y' :0.070677} 
        quaternion1 = {'r1' :-0.000, 'r2' : -0.000, 'r3' : 0.960, 'r4' : 0.281} #run command : rosrun tf tf_echo /map /base_link
        
        
        
        again = 0
        nav_node = TurnRobotNode()
        # result1 = nav_node.to_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
        flag = False
        while(again<3):
      
            result1 = False
            result1 = nav_node.to_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
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
