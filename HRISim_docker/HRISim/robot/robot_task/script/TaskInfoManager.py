#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from robot_msgs.msg import TasksInfo, TaskInfo
from robot_srvs.srv import NewTask, NewTaskResponse, FinishTask, FinishTaskResponse

class RobotTaskManager():
    def __init__(self):
        self.current_taskID = -1

        # ROS publishers
        self.tasks_info_pub = rospy.Publisher('/hrisim/robot_tasks_info', TasksInfo, queue_size=10)

        # Initialize TaskInfo message
        self.tasks_msg = TasksInfo()
        self.tasks_msg.header = Header()
        self.tasks_msg.Tasks = []
        
        # Define ROS services
        rospy.Service('/hrisim/new_task', NewTask, self.new_task_cb)
        rospy.Service('/hrisim/finish_task', FinishTask, self.finish_task_cb)
    
    @property        
    def next_taskID(self):
        return self.current_taskID + 1
        
        
    def publish_tasks(self):        
        self.tasks_info_pub.publish(self.tasks_msg)
        
        
    def new_task_cb(self, req):
        task = TaskInfo()
        task.task_id = self.next_taskID
        task.path = req.path
        task.final_destination = task.path[-1]
        task.start_time = rospy.Time.now()
        task.result = 0  # 0: WIP, 1: SUCCESS, -1: FAILURE
        self.tasks_msg.Tasks.append(task)
        self.tasks_msg.header.stamp = rospy.Time.now()
        self.current_taskID += 1
        
        return NewTaskResponse(task.task_id)
        
        
    def finish_task_cb(self, req):
        for task in self.tasks_msg.Tasks:
            if task.task_id == req.task_id:
                task.end_time = rospy.Time.now()
                task.result = req.result
                return FinishTaskResponse(True)
        return FinishTaskResponse(False)


if __name__ == '__main__':
    rospy.init_node('robot_taskmanager')
    rate = rospy.Rate(10)
    
    RTM = RobotTaskManager()
    
    while not rospy.is_shutdown():
        RTM.publish_tasks()
        rate.sleep()