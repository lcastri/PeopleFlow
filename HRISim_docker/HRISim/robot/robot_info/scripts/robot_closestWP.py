#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import hrisim_util.ros_utils as ros_utils


class RobotClosestWP():
    def __init__(self) -> None:
        rospy.Subscriber("/robot_pose" , PoseWithCovarianceStamped, self.cb_robot_pose)
        self.wp_pub = rospy.Publisher('/hrisim/robot_closest_wp', String, queue_size=10)  
    
    def cb_robot_pose(self, pose: PoseWithCovarianceStamped):
        potential_wp = {}
        x, y, _ = ros_utils.getPose(pose.pose.pose)
        for wp in WPS:
            d = math.sqrt((WPS[wp]['x'] - x)**2 + (WPS[wp]['y'] - y)**2)
            potential_wp[wp] = d
        wp = min(potential_wp, key=potential_wp.get)
        self.wp_pub.publish(String(wp))

    
if __name__ == '__main__':
    rospy.init_node('robot_closest_wp')
    rate = rospy.Rate(10)  # 1 Hz
    SCENARIO = str(ros_utils.wait_for_param("/peopleflow_manager/scenario"))
    WPS = ros_utils.wait_for_param("/peopleflow/wps")

    RWPP = RobotClosestWP()
    
    rospy.spin()