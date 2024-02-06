#!/usr/bin/env python
# 
# CSCE 839 HW2 Sung Woo Choi

import roslib
import rospy
import ctypes
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Float64
from random import gauss

np.set_printoptions(formatter={'float': '{: 0.10f}'.format})

class Flags_bits(ctypes.LittleEndianStructure):
    _fields_ = [
        ("tAlt", ctypes.c_uint8, 1),
        ("tVel", ctypes.c_uint8, 1),
        ("tAcl", ctypes.c_uint8, 1),
        ("sVel", ctypes.c_uint8, 1),
        ("sAcl", ctypes.c_uint8, 1),
        ("cAcl", ctypes.c_uint8, 1),
    ]
    
class Flags(ctypes.Union):
    _fields_ = [("b", Flags_bits),
                ("asbyte", ctypes.c_uint8)]

class KalmanFiter(object):
    def __init__(self):
        rospy.init_node('kalmanFiter')
        # NOTE: Actual variables (groundtruth/altitude, groundtruth/velocity, groundtruth/acceleration) are
        #       received via subscribers, however, they are not used in Kalman Filter. These variables are 
        #       recieved to prevent rqt_plot not receiving actual variables on time. Also, these variables are
        #       recieved to compare variables computed from Kalman Filter to actual variables for debugging 
        #       purposes.
        
        # Create Subscribers
        self.sub_true_altitude = rospy.Subscriber('groundtruth/altitude', Float64, self.trueAltitude_cb)
        self.sub_true_vel = rospy.Subscriber('groundtruth/velocity', Float64, self.trueVelocity_cb)
        self.sub_true_accel = rospy.Subscriber('groundtruth/acceleration', Float64, self.trueAcceleration_cb)
        self.sub_sensed_accel = rospy.Subscriber('sensors/acceleration', Float64, self.sensedAccel_cb)
        self.sub_sensed_vel = rospy.Subscriber('sensors/velocity', Float64, self.sensedVel_cb)
        self.sub_control_accel = rospy.Subscriber('control/acceleration', Float64, self.controlAccel_cb)
        # Create Publishers
        self.pub_kalman_altitude = rospy.Publisher('kalman/altitude', Float64, queue_size=1)
        self.pub_kalman_vel = rospy.Publisher('kalman/velocity', Float64, queue_size=1)
        self.pub_kalman_accel = rospy.Publisher('kalman/acceleration', Float64, queue_size=1)
        # Initiate publish message variables
        self.k_altitude_msg = Float64()
        self.k_vel_msg = Float64()
        self.k_accel_msg = Float64()
        # Initiate flags to check if all variables are received from subscribers
        self.flags = Flags()
        self.flags.asbyte = 0x00
        
        # Kalman filter variables
        # A related prior state to current state
        self.A = np.array([[1.0,1.0,0.5],[0.0,1.0,1.0],[0.0,0.0,1.0]])
        # H relates state to measurement
        self.H = np.array([[0.0,1.0,0.0],[0.0,0.0,1.0]])
        # B maps control input u to state x
        self.B = np.array([[0.0],[0.0],[1.0]])
        
        # Q process noise covariance
        self.Q = np.diag([0.01, 0.01, 1.0]) 
        # R measurement noise convariance
        self.R = np.diag([0.01, 1.0])
        
        # Kalman filter initalaization
        self.x = np.array([[0.0],[0.0],[0.0]])
        self.z = np.array([[0.0],[0.0]])
        self.u = 0
        # P is prior error covaraince (error in estimate of x before intergarting new measurement)
        self.P   = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.P_k = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
            
        # Column vector for actual variables for debugging purposes
        self.x_true = np.array([[0.0],[0.0],[0.0]])
            
    def trueAltitude_cb(self, true_altitude):
        self.x_true[0] = true_altitude.data
        self.flags.b.tAlt = 1
        
    def trueVelocity_cb(self, true_vel):
        self.x_true[1] = true_vel.data
        self.flags.b.tVel = 1

    def trueAcceleration_cb(self, true_accel):
        self.x_true[2] = true_accel.data
        self.flags.b.tAcl = 1
        
    def sensedVel_cb(self, sensed_vel):
        self.z[0] = sensed_vel.data
        self.flags.b.sVel = 1
        
    def sensedAccel_cb(self, sensed_accel):
        self.z[1] = sensed_accel.data
        self.flags.b.sAcl = 1
        
    def controlAccel_cb(self, control_accel):
        self.u = control_accel.data
        self.flags.b.cAcl = 1
    
    def processKalmanFilter(self):
        while(self.flags.asbyte != 0x3F):
            self.x = np.array([[0],[self.z[0]],[self.z[1]]])
            self.x_k = np.array([[0],[self.z[0]],[self.z[1]]])
            break
        self.flags.asbyte = 0x00
        while not rospy.is_shutdown():
            if self.flags.asbyte == 0x3F:
                # received all variables from rocketShip.
                # process Kalman Filter computation
                
                # Time update
                self.x_k = np.matmul(self.A, self.x) + self.B*self.u
                T = np.matmul(self.P, self.A.transpose())
                self.P_k = np.matmul(self.A, T) + self.Q
                                
                # Measurement update
                T = np.matmul(self.P_k, self.H.transpose())
                T = np.linalg.inv(np.matmul(self.H, T) + self.R)
                T = np.matmul(self.H.transpose(), T)
                self.K = np.matmul(self.P_k, T)
                self.x = self.x_k + np.matmul(self.K, (self.z - np.matmul(self.H, self.x_k)))
                self.P = np.matmul((np.eye(self.H.shape[1]) - np.matmul(self.K, self.H)), self.P_k)
                self.flags.asbyte = 0x00
                
                # Populate messages and publish
                self.k_altitude_msg.data = self.x[0]
                self.k_vel_msg.data = self.x[1]
                self.k_accel_msg.data =  self.x[2]
                self.pub_kalman_altitude.publish(self.k_altitude_msg)
                self.pub_kalman_vel.publish(self.k_vel_msg)
                self.pub_kalman_accel.publish(self.k_accel_msg)
            rospy.sleep(0.01)

if __name__ == '__main__':
    a = KalmanFiter()
    a.processKalmanFilter()