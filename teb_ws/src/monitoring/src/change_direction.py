import rospy
import math
from geometry_msgs import Pose, Point, Quaternion
from tb_apala.msg import position

class rotate:
    def __init__(self):
        
        rospy.Subscriber("odom", Pose , self.rob_pose_callback, queue_size=10)
        rospy.Subscriber("position_h1",position,self.position1_callback,queue_size=1)
        rospy.Subscriber("position_h2",position,self.position2_callback,queue_size=1)
        
    
    def rob_pose_callback(self, msg):
        self.quart = msg.pose.pose.orientation.w
        self.rad = 2*(math.acos(self.quart)) # orientation.w = q[0] = cos(rad/2)         
       
        # x = q[1] = sin(r/2)*x;
        # y = q[2] = sin(r/2)*y;
        # z = q[3] = sin(r/2)*z;
        # w = q[0] = cos(r/2);
        
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        self.rad1 = self.calc_theta(self.pose_x, self.pose_y, self.x_cord1, self.y_cord1)
        self.q1_w = math.cos((math.pi - self.rad1)/2)
        self.q1_x = math.sin((math.pi - self.rad1)/2) * self.x_cord1
        self.q1_y = math.sin((math.pi - self.rad1)/2) * self.y_cord1
        self.q1_z = math.sin((math.pi - self.rad1)/2) * 0.0
        
        
        self.rad2 = self.calc_theta(self.pose_x, self.pose_y, self.x_cord2, self.y_cord2)
        
        
        
        
    def position1_callback(self,data): 
        self.x_cord1 = data.x
        self.y_cord1 = data.z
        
        
    def position2_callback(self,data): 
        self.x_cord2 = data.x
        self.y_cord2 = data.z
        
        
    def calc_theta(self, x1, y1, x2, y2):
        # y = mx + c
        # m  = y2-y2/x2-x1
        
        theta = ((self.y2 - self.y1 ) / (self.x2 - self.x1))
        rad = math.radians(theta)
        return rad
        
        