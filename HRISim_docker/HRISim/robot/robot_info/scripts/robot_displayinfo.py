#!/usr/bin/env python

import rospy
import hrisim_util.ros_utils as ros_utils
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Bool, Float32, String
from nav_msgs.msg import Odometry


def cb_closest_wp(msg: String):
    global CLOSEST_WP    
    CLOSEST_WP = msg.data
    
          
def cb_battery(msg: Float32):
    global BATTERY_LEVEL    
    BATTERY_LEVEL = msg.data        
    
    
def cb_mode(msg: Bool):
    global MODE
    MODE = "MANUAL" if msg.data else "AUTONOMOUS"
    
    
def cb_vel(msg: Odometry):
    global ROBOT_VEL
    ROBOT_VEL = msg.twist.twist.linear.x
    
 
def create_overlay_text():
    
    text_main = OverlayText()
    text_main.width = 400  # Width of the overlay
    text_main.height = 115  # Height of the overlay
    text_main.left = 10  # X position (left offset)
    text_main.top = 10  # Y position (top offset)
    text_main.text_size = 13  # Font size
    text_main.line_width = 2
    mode_info = MODE if MODE is not None else 'none'
    battery_info = f"{BATTERY_LEVEL:.2f}%" if BATTERY_LEVEL is not None else 'none'
    vel_info = f"{ROBOT_VEL:.2f}%" if ROBOT_VEL is not None else 'none'
    WP_info = CLOSEST_WP if CLOSEST_WP is not None else 'none'
    intro_str = "TIAGo:"
    mode_str = f"- Mode: {mode_info}"
    battery_str = f"- Battery: {battery_info}"
    velocity_str = f"- Velocity: {vel_info} m/s"
    wp_str = f"- Closest WP: {WP_info}"
    overlay_str = '\n'.join([intro_str, mode_str, battery_str, velocity_str, wp_str])
    text_main.text = overlay_str
    text_main.font = "DejaVu Sans Mono"
    text_main.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)

    return text_main


if __name__ == '__main__':
    rospy.init_node('robot_displayinfo')
    rate = rospy.Rate(10)  # 1 Hz
    
    BATTERY_LEVEL = None
    MODE = None
    ROBOT_VEL = None
    CLOSEST_WP = None
    
    rospy.Subscriber('/power/battery_level', Float32, cb_battery)
    rospy.Subscriber('/joy_priority', Bool, cb_mode)
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, cb_vel)
    rospy.Subscriber('/hrisim/robot_closest_wp', String, cb_closest_wp)

    text_pub = rospy.Publisher('/hrisim/robot/info/main', OverlayText, queue_size=10)
    
    while not rospy.is_shutdown():
        
        # Overlay text
        text_main = create_overlay_text()
        text_pub.publish(text_main)
                
        rate.sleep()