#!/usr/bin/env python

import math
import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
from robot_msgs.msg import BatteryStatus
import hrisim_util.ros_utils as ros_utils

def cb_battery(msg):
    global BATTERY_LEVEL, BATTERY_ISCHARGING
    BATTERY_LEVEL = msg.level.data
    # BATTERY_LEVEL = math.floor(msg.level.data)
        
    BATTERY_ISCHARGING = bool(msg.is_charging.data)
 
def create_overlay_text():
    LOADED = bool(ros_utils.wait_for_param('/hrisim/robot_load'))
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/TebLocalPlannerROS/max_vel_x"))
    text = OverlayText()
    text.width = 400  # Width of the overlay
    text.height = 100  # Height of the overlay
    text.left = 10  # X position (left offset)
    text.top = 35  # Y position (top offset)
    text.text_size = 13  # Font size
    text.line_width = 2
    if BATTERY_LEVEL is not None:
        battery_info = f"{BATTERY_LEVEL:.2f}%" 
        # battery_info = f"{BATTERY_LEVEL}%" 
    else:
        battery_info = 'none'
    text.text = f"Robot\n- Battery: {battery_info}\n- Carrying Load: {'True' if LOADED else 'False'}\n- Max Velocity: {ROBOT_MAX_VEL:.2f} m/s"
    text.font = "DejaVu Sans Mono"
       
    # Text color
    text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
    
    return text

    
if __name__ == '__main__':
    rospy.init_node('robot_displayinfo')
    rate = rospy.Rate(1)  # 1 Hz
    
    BATTERY_LEVEL = None
    BATTERY_ISCHARGING = False
    # starting = rospy.get_time()

    
    rospy.Subscriber('/hrisim/robot_battery', BatteryStatus, cb_battery)
    info_pub = rospy.Publisher('/hrisim/otext_robot_info', OverlayText, queue_size=10)
    
    while not rospy.is_shutdown():
        
        # Overlay text
        msg = create_overlay_text()
        info_pub.publish(msg)
                
                
        rate.sleep()