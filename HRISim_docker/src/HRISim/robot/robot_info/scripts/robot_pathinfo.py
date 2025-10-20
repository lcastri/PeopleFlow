#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray as G
import math
from robot_msgs.msg import PathInfo as PI

class PathInfo():
    def __init__(self) -> None:
        self.status = None
        self.pstatus = -1
        self.received = None
        self.elapsed = None
        self.path_length = 0
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.cb_pathlen)
        rospy.Subscriber('/move_base/status', G, self.cb_status)
        self.info_pub = rospy.Publisher('/hrisim/robot_pathinfo', PI, queue_size=10)

    def cb_status(self, msg):
        if msg.status_list:
            self.pstatus = self.status
            self.status = msg.status_list[-1].status
            if self.pstatus != self.status and self.status == 3 and self.received is not None: 
                self.elapsed = rospy.get_time() - self.received
                
                msg = PI()
                msg.status = self.status
                msg.path_length = self.path_length
                msg.reaching_time = self.elapsed
                self.info_pub.publish(msg)

    def cb_pathlen(self, plan):
        if self.pstatus != self.status:
            self.received = rospy.get_time()
            self.elapsed = None
            self.path_length = self.compute_path_length(plan)


    def compute_path_length(self, path):
        length = 0.0
        for i in range(len(path.poses) - 1):
            x1 = path.poses[i].pose.position.x
            y1 = path.poses[i].pose.position.y
            x2 = path.poses[i+1].pose.position.x
            y2 = path.poses[i+1].pose.position.y
            # Compute the Euclidean distance between consecutive points
            length += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return length


    
if __name__ == '__main__':
    rospy.init_node('robot_pathinfo')
    rate = rospy.Rate(1)  # 1 Hz
        
    P = PathInfo()
    rospy.spin()