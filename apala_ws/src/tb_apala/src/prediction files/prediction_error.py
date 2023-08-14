
"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot in Construction site
    Advised by: Dr.Dung Hoang Tran, Dr.Kyungki Kim
    
    
"""


import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
from tb_apala.msg import position
from visualization_msgs.msg import Marker
from marker_publisher import marker
from eval_pred import FilterEstimator
from std_msgs.msg import Float32MultiArray
from cv2 import HuMoments
import rospy
from cmath import isnan, nan, sqrt
from os import device_encoding
from numpy import NaN, cov, poly

class prediction_error():
    def __init__(self) :
        rospy.Subscriber("pred1_array",Float32MultiArray,self.pred1_callback,queue_size=10)
        rospy.Subscriber("pred2_array",Float32MultiArray,self.pred2_callback,queue_size=10)
        rospy.Subscriber("position_h1", position, self.position1_callback,queue_size=10 )
        rospy.Subscriber("position_h2", position, self.position2_callback,queue_size=10 )

    def calculate_error():
        
