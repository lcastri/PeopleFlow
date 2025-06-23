#!/usr/bin/env python

import rospy
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Bool, Float32, String
from nav_msgs.msg import Odometry
from hrisim_people_counter.msg import WPPeopleCounters


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
    
    
def cb_people_counter(msg: WPPeopleCounters):
    global MAX_WP
    counters = {counter.WP_id.data: counter.numberOfPeople for counter in msg.counters}
    max_wp = max(counters, key=counters.get)
    MAX_WP = f"{max_wp}\t{counters[max_wp]}"
 
def create_overlay_text():
    global COLORED
    TS = rospy.get_param("/hrisim/time_slot")
    
    text_main = OverlayText()
    text_main.width = 250  # Width of the overlay
    text_main.height = 145  # Height of the overlay
    text_main.left = 10  # X position (left offset)
    text_main.top = 10  # Y position (top offset)
    text_main.text_size = 13  # Font size
    text_main.line_width = 2
    mode_info = MODE if MODE is not None else 'none'
    battery_info = f"{BATTERY_LEVEL:.2f}%" if BATTERY_LEVEL is not None else 'none'
    vel_info = f"{ROBOT_VEL:.2f}%" if ROBOT_VEL is not None else 'none'
    WP_info = CLOSEST_WP if CLOSEST_WP is not None else 'none'
    MAX_WP_info = MAX_WP if MAX_WP is not None else 'none'
    scenario_str = f"Scenario: {constants.SCENARIOS[TS]['name']}"
    intro_str = "TIAGo:"
    mode_str = f"- Mode: {mode_info}"
    battery_str = f"- Battery: {battery_info}"
    velocity_str = f"- Velocity: {vel_info} m/s"
    wp_str = f"- Closest WP: {WP_info}"
    max_wp_str = f"- Busiest WP: {MAX_WP_info}"
    overlay_str = '\n'.join([scenario_str, intro_str, mode_str, battery_str, velocity_str, wp_str, max_wp_str])
    text_main.text = overlay_str
    text_main.font = "DejaVu Sans Mono"
    text_main.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
    
    # Set background color based on MODE
    if MODE == "MANUAL":
        if not COLORED:
            text_main.bg_color = ColorRGBA(255/255,88/255,14/255, 0.5)  # Red with 0.5 opacity
            COLORED = True
        else:
            COLORED = False
        
    return text_main


if __name__ == '__main__':
    rospy.init_node('robot_displayinfo')
    rate = rospy.Rate(2)  # 1 Hz
    
    BATTERY_LEVEL = None
    MODE = None
    ROBOT_VEL = None
    CLOSEST_WP = None
    MAX_WP = None
    COLORED = False
    
    ros_utils.wait_for_param("/hrisim/time_slot")
    rospy.Subscriber('/power/battery_level', Float32, cb_battery)
    rospy.Subscriber('/joy_priority', Bool, cb_mode)
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, cb_vel)
    rospy.Subscriber('/hrisim/robot_closest_wp', String, cb_closest_wp)
    rospy.Subscriber('/hrisim/people_counter', WPPeopleCounters, cb_people_counter)

    text_pub = rospy.Publisher('/hrisim/robot/info/main', OverlayText, queue_size=10)
    
    while not rospy.is_shutdown():
        
        # Overlay text
        text_main = create_overlay_text()
        text_pub.publish(text_main)
                
        rate.sleep()