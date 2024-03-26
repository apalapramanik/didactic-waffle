
    
#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg  # Import nav_msgs for Odometry


def handle_tb3_pose(msg, robot_name):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"  # Use "odom" as the parent frame
    t.child_frame_id = robot_name
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtlebot3_broadcaster')
    robot_name = rospy.get_param('~tb3')
    rospy.Subscriber('/%s/odom' % robot_name,  # Subscribe to Odometry topic
                     nav_msgs.msg.Odometry,  # Use Odometry message type
                     handle_tb3_pose,
                     robot_name)
    rospy.spin()
    
    
