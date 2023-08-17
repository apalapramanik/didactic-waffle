import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion
from tb_apala.msg import position
from tb_apala.msg import distance
from cmath import isnan, nan
from std_msgs.msg import Float32


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
        self.min_ttc = 0.09
        self.speed_h1 = 0.0
        self.prev_point = None
        self.prev_time = None
        
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber("move_base/feedback",MoveBaseActionFeedback,self.feed_cb,queue_size=10)
        rospy.Subscriber("position_h1",position,self.position1_callback,queue_size=10)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        
        self.distance_human1 = rospy.Publisher("distance_from_human1", distance,queue_size=1)
        self.speed_human1 = rospy.Publisher('/human1_speed', Float32, queue_size=10)
    
    def position1_callback(self,data): 
          
            
        if self.prev_point is not None:
            current_time = rospy.Time.now()
            # time_difference = (current_time - self.prev_time).to_nsec()
            # print(time_difference)

            distance_h1 = self.calculate_distance(self.prev_point, data)
            print("distance: ", distance_h1)
            self.speed_h1 = distance_h1 / 0.01
            print("speed: ", self.speed_h1)

            # self.speed_human1.publish(self.speed_h1)
            # print("human_speed:", self.speed_h1)

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
        distance = math.sqrt(dx**2 + dy**2 + 0)
        return distance

    def odom_callback(self, odom_msg):
        orientation_quaternion = odom_msg.pose.pose.orientation
        self.current_orientation = math.atan2(2.0 * (orientation_quaternion.w * orientation_quaternion.z + orientation_quaternion.x * orientation_quaternion.y),
                                              1.0 - 2.0 * (orientation_quaternion.y * orientation_quaternion.y + orientation_quaternion.z * orientation_quaternion.z))
        
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
        
        if self.dist1 <1.5:
            rospy.loginfo("waiting")

        self.move_base_client.send_goal(turn_goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Robot turned by {angle_degrees} degrees successfully.")
            self.goal2 = True
        else:
            rospy.logwarn("Turning the robot failed.")
            self.goal2 = False
            
        return self.goal2

 
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
        success = self.move_base_client.wait_for_result()
        state = self.move_base_client.get_state() 

        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            # rospy.loginfo("reached location yayyyy  ")
            self.goal1= True
        else:
            # rospy.logwarn("couldnt make it :((((  ")
            self.goal1 = False
      
        return self.goal1
    
    
    def feed_cb(self,feedback):  
            
        # print("here in feedback")
        self.ttc_1 = self.time_to_collision(self.dist1, self.speed_h1)
        print("TTC: ", self.ttc_1)
        if self.ttc_1 < self.min_ttc:
            self.stop()
            print("cancelled goal")
            

           
           
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
        

      
        nav_node = TurnRobotNode()
        result1 = nav_node.to_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
        if result1:
            rospy.loginfo("reached destination")
        else:
            rospy.loginfo("changing direction")
            
            result2 = nav_node.turn_robot(45.0)
            if result2:
                # result3 = nav_node.to_goal(position2, quaternion2, orientation2[0],orientation2[1], orientation2[2])
                result3 = nav_node.to_goal(position1, quaternion1, orientation1[0],orientation1[1], orientation1[2])
        
    except rospy.ROSInterruptException:
        pass
