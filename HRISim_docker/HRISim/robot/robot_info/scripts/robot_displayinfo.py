#!/usr/bin/env python

import rospy
import hrisim_util.ros_utils as ros_utils
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Bool, Int32
from robot_msgs.msg import BatteryStatus, TasksInfo
from nav_msgs.msg import Odometry

def cb_battery(msg):
    global BATTERY_LEVEL, BATTERY_ISCHARGING
    BATTERY_LEVEL = msg.level.data        
    BATTERY_ISCHARGING = bool(msg.is_charging.data)
    
    
def cb_obs(msg: Bool):
    global OBS
    OBS = msg.data    
    
        
def cb_vel(odom):
    global ROBOT_VEL
    ROBOT_VEL = abs(odom.twist.twist.linear.x)
    
    
def cb_robot_tasks(msg: TasksInfo):      
    global TASKS
    TASKS = (int(msg.num_tasks), int(msg.num_success), int(msg.num_failure))
    
    
def cb_robot_human_collision(msg: Int32):
    global COLLISION
    COLLISION += int(msg.data)
    
 
def create_overlay_text():
    
    # Battery and Velocity
    text_main = OverlayText()
    text_main.width = 400  # Width of the overlay
    text_main.height = 115  # Height of the overlay
    text_main.left = 10  # X position (left offset)
    text_main.top = 35  # Y position (top offset)
    text_main.text_size = 13  # Font size
    text_main.line_width = 2
    battery_info = f"{BATTERY_LEVEL:.2f}%" if BATTERY_LEVEL is not None else 'none'
    vel_info = f"{ROBOT_VEL:.2f}%" if ROBOT_VEL is not None else 'none'
    intro_str = "Info:"
    battery_str = f"- Battery: {battery_info}"
    velocity_str = f"- Velocity: {vel_info} m/s"
    collision_str = f"- Collisions: {COLLISION}"
    overlay_str = '\n'.join([intro_str, battery_str, velocity_str, collision_str])
    text_main.text = overlay_str
    text_main.font = "DejaVu Sans Mono"
    text_main.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
    
    # Obstacle
    text_obstacle = OverlayText()
    text_obstacle.width = 400  # Width of the overlay
    text_obstacle.height = 30  # Height of the overlay
    text_obstacle.left = 10  # X position (left offset)
    text_obstacle.top = text_main.height # Y position (top offset)
    text_obstacle.text_size = 13  # Font size
    text_obstacle.line_width = 2
    text_obstacle.text = f"- Obstacle: {'True' if OBS else 'False'}"
    text_obstacle.font = "DejaVu Sans Mono"
    text_obstacle.fg_color = ColorRGBA(1.0, 0.0, 0.0, 1.0) if OBS else ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Red if True, Green if False
    
    # Battery and Velocity
    text_task = OverlayText()
    text_task.width = 400  # Width of the overlay
    text_task.height = 120  # Height of the overlay
    text_task.left = 10  # X position (left offset)
    text_task.top = text_main.height + text_obstacle.height + 5  # Y position (top offset)
    text_task.text_size = 13  # Font size
    text_task.line_width = 2
    if TASKS is not None:
        intro_str = f"Task {TASKS[0]}/{int(rospy.get_param('/hrisim/tasks/total', 0))}:"
        tasks_detail_str = f"- Pending: {TASKS[0] - (TASKS[1]+TASKS[2])}\n- Success: {TASKS[1]}/{int(TASKS[0])-1}\n- Failure: {TASKS[2]}/{int(TASKS[0])-1}" if TASKS is not None else 'none'
        overlay_str = '\n'.join([intro_str, tasks_detail_str])
    else:
        overlay_str = "Task none"
    text_task.text = overlay_str
    text_task.font = "DejaVu Sans Mono"
    text_task.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
       
    
    return text_main, text_obstacle, text_task


if __name__ == '__main__':
    rospy.init_node('robot_displayinfo')
    rate = rospy.Rate(1)  # 1 Hz
    
    BATTERY_LEVEL = None
    COLLISION = 0
    BATTERY_ISCHARGING = False
    OBS = False
    ROBOT_VEL = None
    TASKS = None
    
    rospy.Subscriber('/hrisim/robot_battery', BatteryStatus, cb_battery)
    rospy.Subscriber('/hrisim/robot_obs', Bool, cb_obs)
    rospy.Subscriber('/hrisim/robot_tasks_info', TasksInfo, cb_robot_tasks)
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, cb_vel)
    rospy.Subscriber("/hrisim/robot_human_collision", Int32, cb_robot_human_collision)

    text_pub = rospy.Publisher('/hrisim/robot/info/main', OverlayText, queue_size=10)
    task_pub = rospy.Publisher('/hrisim/robot/info/tasks', OverlayText, queue_size=10)
    obstacle_pub = rospy.Publisher('/hrisim/robot/info/obstacle', OverlayText, queue_size=10)
    
    while not rospy.is_shutdown():
        
        # Overlay text
        text_main, text_obstacle, text_task = create_overlay_text()
        text_pub.publish(text_main)
        obstacle_pub.publish(text_obstacle)
        task_pub.publish(text_task)
                
        rate.sleep()