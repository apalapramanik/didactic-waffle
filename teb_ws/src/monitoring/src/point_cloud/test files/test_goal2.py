
import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TurnRobotNode:
    def __init__(self):
        rospy.init_node('turn_robot_node')
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def turn_robot(self, angle_degrees):
        turn_goal = MoveBaseGoal()
        turn_goal.target_pose.header.frame_id = 'base_link'
        turn_goal.target_pose.header.stamp = rospy.Time.now()

        target_angle = angle_degrees * (math.pi / 180.0)
        turn_goal.target_pose.pose.orientation.z = math.sin(target_angle / 2.0)
        turn_goal.target_pose.pose.orientation.w = math.cos(target_angle / 2.0)

        self.move_base_client.send_goal(turn_goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Robot turned by {angle_degrees} degrees successfully.")
        else:
            rospy.logwarn("Turning the robot failed.")



if __name__ == '__main__':
    try:
        turn_node = TurnRobotNode()
        turn_node.turn_robot(30.0)  # Turn by 30 degrees
        
    except rospy.ROSInterruptException:
        pass
