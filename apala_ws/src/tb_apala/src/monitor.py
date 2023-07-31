"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot in Construction site
    Advised by: Dr.Dung Hoang Tran, Dr.Kyungki Kim
    
    
"""

#!/usr/bin/env python
#!/usr/bin/env python3

#imports 
# import sys
# import time
# import ros_numpy
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
# from tb_apala.msg import MinMax
from std_msgs.msg import String


class operator:
    
    def __init__(self):
        
        
        rospy.Subscriber("cp_flag", String, self.cp_flag_callback, queue_size=10 )
        rospy.Subscriber("cmd_vel", Twist,self.vel_callback,queue_size=10) 
        rospy.Subscriber("pred1_array",Float32MultiArray,self.pred1_callback,queue_size=10)
        rospy.Subscriber("pred2_array",Float32MultiArray,self.pred2_callback,queue_size=10)
        # rospy.Subscriber("pred3_array",Float32MultiArray,self.pred3_callback,queue_size=10)
        
        # self.total_velocity = rospy.Publisher("velocity", Float32,queue_size=10)
        self.always1 = rospy.Publisher("always_human1", Float32,queue_size=10)
        self.always2 = rospy.Publisher("always_human2", Float32,queue_size=10)
        # self.always3 = rospy.Publisher("always_human3", Float32,queue_size=10)
        
        self.eventually1 = rospy.Publisher("eventually_dist1", Float32,queue_size=10)
        self.eventually2 = rospy.Publisher("eventually_dist2", Float32,queue_size=10)
        # self.eventually3 = rospy.Publisher("eventually_dist3", Float32,queue_size=10)
        
        self.implies1 = rospy.Publisher("halting_h1", Float32,queue_size=10)
        self.implies2 = rospy.Publisher("halting_h2", Float32,queue_size=10)
        # self.implies3 = rospy.Publisher("halting_h3", Float32,queue_size=10)
        
        self.until1 = rospy.Publisher("human_h1", Float32,queue_size=10)
        self.until2 = rospy.Publisher("human_h2", Float32,queue_size=10)
        # self.until3 = rospy.Publisher("human_h3", Float32,queue_size=10)
        
        self.velocity = 0.0
        self.turtle_vel = rospy.Publisher("velocity", Float32,queue_size=10)
        turtle_vel_msg = Float32()
        if self.velocity:
            turtle_vel_msg.data = self.velocity
        else:
            turtle_vel_msg.data = 0.0
        self.turtle_vel.publish(turtle_vel_msg)
            
        
        
    def cp_flag_callback(self, msg):
        self.flag = msg.data
        
     
    def vel_callback(self, data):
        self.vel_x = data.linear.x
        self.vel_y = data.linear.y
        self.vel_z = data.linear.z
        
        self.velocity = (self.vel_x**2 + self.vel_y**2 + self.vel_z**2)**0.5
        self.turtle_vel.publish(self.velocity)   
        
    def pred1_callback(self, msg):
        self.pred_dist1 = list(msg.data)
        
        #always operator
        self.result_array1 = [value - 1 for value in self.pred_dist1]
        self.min_val1 = min(self.result_array1)
        always_msg1 = Float32()
        always_msg1.data = self.min_val1
        self.always1.publish(always_msg1)
        
        with open('always_human1.txt', 'a') as file:
            file.write(str(self.min_val1) + '\n')

        
        
        #eventually operator
        self.result_array1 = [1 - value for value in self.pred_dist1]
        self.max_val1 = max(self.result_array1)
        eventually_msg1 = Float32()
        eventually_msg1.data = self.max_val1
        self.eventually1.publish(eventually_msg1)
        
        with open('eventually_human1.txt', 'a') as file:
            file.write(str(self.max_val1) + '\n')

       
        
        #implies operator
        if self.velocity:
            pass
        else:
            self.velocity = 0.0
        self.spec3a = max(-self.max_val1, self.velocity)
        implies_msg1 = Float32()
        implies_msg1.data = self.spec3a
        self.implies1.publish(implies_msg1)
        with open('implies_human1.txt', 'a') as file:
            file.write(str(self.max_val1) + '\n')
        
        
        
        #until operator:
        # self.until_op1 = min(self.min_val1,self.max_val1)
        # until_msg1 = Float32()
        # until_msg1.data = self.until_op1
        # self.until1.publish(until_msg1)
        
        #until operator:
        index1 = next((i for i, x in enumerate(self.pred_dist1) if x <= 1.25), None)
        if index1 is not None:
            # calculate max of values after index
            max_after1 = max(1 - value for value in self.pred_dist1[index1:])
            until_msg1 = Float32()            
            until_msg1.data = max_after1
            self.until1.publish(until_msg1)
            
            with open('until_human1.txt', 'a') as file:
                file.write(str(max_after1) + '\n')
            
        else: 
            # calculate min of values before index
            min_before1 = min(value - 1.0 for value in self.pred_dist1)
            until_msg1 = Float32()
            until_msg1.data = min_before1
            self.until1.publish(until_msg1)
            with open('until_human1.txt', 'a') as file:
                file.write(str(min_before1) + '\n')
            
       

        
            
        
    
    def pred2_callback(self, msg):
        self.pred_dist2 = list(msg.data)
        
        #always operator
        self.result_array2 = [value - 1.0 for value in self.pred_dist2]
        self.min_val2 = min(self.result_array2)
        always_msg2 = Float32()
        always_msg2.data = self.min_val2
        self.always2.publish(always_msg2)
        with open('always_human2.txt', 'a') as file:
            file.write(str(self.min_val2) + '\n')
            
        
        
        #eventually_operator
        self.result_array2 = [1.25 - value for value in self.pred_dist2]
        self.max_val2 = max(self.result_array2)
        eventually_msg2 = Float32()
        eventually_msg2.data = self.max_val2
        self.eventually2.publish(eventually_msg2)
        with open('eventually_human2.txt', 'a') as file:
            file.write(str(self.max_val2) + '\n')
            
            
            
        
        #implies operator
        if self.velocity:
            pass
        else:
            self.velocity = 0.0
        self.spec3b = max(-self.max_val2, self.velocity)
        implies_msg2 = Float32()
        implies_msg2.data = self.spec3b
        self.implies2.publish(implies_msg2)
        with open('implies_human2.txt', 'a') as file:
            file.write(str(self.spec3b) + '\n')
            
        
        #until operator:
        # self.until_op2 = min(self.min_val2,self.max_val2)
        # until_msg2 = Float32()
        # until_msg2.data = self.until_op2
        # self.until1.publish(until_msg2)
        
        #until operator:
        index2 = next((i for i, x in enumerate(self.pred_dist2) if x <= 1.25), None)
        
        if index2 is not None:
            # calculate max of values after index
            max_after2 = max(1- value for value in self.pred_dist1[index2:])
            until_msg2 = Float32()            
            until_msg2.data = max_after2
            self.until2.publish(until_msg2)
            with open('until_human2.txt', 'a') as file:
                file.write(str(max_after2) + '\n')
            
            
            
        else: 
            # calculate min of values before index
            min_before2 = min(value - 1 for value in self.pred_dist2)
            until_msg2 = Float32()
            until_msg2.data = min_before2
            self.until2.publish(until_msg2)
            with open('until_human2.txt', 'a') as file:
                file.write(str(min_before2) + '\n')
            
            
    # def pred3_callback(self, msg):
    #     self.pred_dist3 = list(msg.data)  
         

    #     #always operator
    #     self.result_array3 = [value - 1.25 for value in self.pred_dist3]
    #     self.min_val3 = min(self.result_array3)
    #     always_msg3 = Float32()
    #     always_msg3.data = self.min_val3
    #     self.always3.publish(always_msg3)
        
    #     #eventually operator   
    #     self.result_array3 = [1.25 - value for value in self.pred_dist3]
    #     self.max_val3 = max(self.result_array3)
    #     eventually_msg3 = Float32()
    #     eventually_msg3.data = self.max_val3
    #     self.eventually3.publish(eventually_msg3)
        
    #     #implies operator
    #     if self.velocity:
    #         pass
    #     else:
    #         self.velocity = 0.0
    #     self.spec3c = max(-self.max_val3, self.velocity)
    #     implies_msg3 = Float32()
    #     implies_msg3.data = self.spec3c
    #     self.implies3.publish(implies_msg3)
        
    #     #until operator:
    #     # self.until_op3 = min(self.min_val3,self.max_val3)
    #     # until_msg3 = Float32()
    #     # until_msg3.data = self.until_op3
    #     # self.until1.publish(until_msg3)
        
    #     #until operator:
    #     index3 = next((i for i, x in enumerate(self.pred_dist3) if x <= 1.25), None)
        

    #     if index3 is not None:
    #         # calculate max of values after index
    #         max_after3 = max(1.25 - value for value in self.pred_dist1[index3:])
    #         until_msg3 = Float32()            
    #         until_msg3.data = max_after3
    #         self.until3.publish(until_msg3)
            
    #     else: 
    #         # calculate min of values before index
    #         min_before3 = min(value - 1.25 for value in self.pred_dist1)
    #         until_msg3 = Float32()
    #         until_msg3.data = min_before3
    #         self.until3.publish(until_msg3)
 
   
    
def main():

    rospy.init_node('monitoring_node', anonymous=False)     
    mon =  operator()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()