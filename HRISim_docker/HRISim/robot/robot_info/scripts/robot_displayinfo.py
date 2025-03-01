#!/usr/bin/env python

import rospy
import hrisim_util.ros_utils as ros_utils
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Bool
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
 
 
def create_overlay_text():
    
    # Battery and Velocity
    text_main = OverlayText()
    text_main.width = 400  # Width of the overlay
    text_main.height = 130  # Height of the overlay
    text_main.left = 10  # X position (left offset)
    text_main.top = 35  # Y position (top offset)
    text_main.text_size = 13  # Font size
    text_main.line_width = 2
    battery_info = f"{BATTERY_LEVEL:.2f}%" if BATTERY_LEVEL is not None else 'none'
    vel_info = f"{ROBOT_VEL:.2f}%" if ROBOT_VEL is not None else 'none'
    info_str = "Info:"
    battery_str = f"- Battery: {battery_info}"
    velocity_str = f"- Velocity: {vel_info} m/s"
    tasks_str = f"- Success: {TASKS[1]}/{TASKS[0]}\n- Failure: {TASKS[2]}/{TASKS[0]}" if TASKS is not None else 'none'
    overlay_str = '\n'.join([info_str, battery_str, velocity_str, tasks_str])
    text_main.text = overlay_str
    text_main.font = "DejaVu Sans Mono"
    text_main.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
       
    # Obstacle
    text_obstacle = OverlayText()
    text_obstacle.width = 400  # Width of the overlay
    text_obstacle.height = 35  # Height of the overlay
    text_obstacle.left = 10  # X position (left offset)
    text_obstacle.top = text_main.height + 5  # Y position (top offset)
    text_obstacle.text_size = 13  # Font size
    text_obstacle.line_width = 2
    text_obstacle.text = f"- Obstacle: {'True' if OBS else 'False'}"
    text_obstacle.font = "DejaVu Sans Mono"
    text_obstacle.fg_color = ColorRGBA(1.0, 0.0, 0.0, 1.0) if OBS else ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Red if True, Green if False
    
    return text_main, text_obstacle


if __name__ == '__main__':
    rospy.init_node('robot_displayinfo')
    rate = rospy.Rate(1)  # 1 Hz
    
    BATTERY_LEVEL = None
    BATTERY_ISCHARGING = False
    OBS = False
    ROBOT_VEL = None
    TASKS = None
    
    rospy.Subscriber('/hrisim/robot_battery', BatteryStatus, cb_battery)
    rospy.Subscriber('/hrisim/robot_obs', Bool, cb_obs)
    rospy.Subscriber('/hrisim/robot_tasks_info', TasksInfo, cb_robot_tasks)
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, cb_vel)
    text_pub = rospy.Publisher('/hrisim/robot/info/main', OverlayText, queue_size=10)
    obstacle_pub = rospy.Publisher('/hrisim/robot/info/obstacle', OverlayText, queue_size=10)
    
    while not rospy.is_shutdown():
        
        # Overlay text
        text_main, text_obstacle = create_overlay_text()
        text_pub.publish(text_main)
        obstacle_pub.publish(text_obstacle)
                
        rate.sleep()