#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
from robot_msgs.msg import BatteryStatus

def cb_battery(msg):
    global BATTERY_LEVEL, BATTERY_ISCHARGING, TASK, starting
    BATTERY_LEVEL = float(msg.level.data)
        
    BATTERY_ISCHARGING = bool(msg.is_charging.data)
    TASK = str(rospy.get_param('/hrisim/robot_task', "none"))

 
def create_overlay_text():
    text = OverlayText()
    text.width = 400  # Width of the overlay
    text.height = 75  # Height of the overlay
    text.left = 10  # X position (left offset)
    text.top = 35  # Y position (top offset)
    text.text_size = 13  # Font size
    text.line_width = 2
    if BATTERY_LEVEL is not None:
        battery_info = f"{BATTERY_LEVEL:.2f}%" 
    else:
        battery_info = 'none'
    text.text = f"Robot\n- Task: {TASK}\n- Battery: {battery_info}"
    text.font = "DejaVu Sans Mono"
       
    # Text color
    text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
    
    return text

    
if __name__ == '__main__':
    rospy.init_node('robot_taskinfo')
    rate = rospy.Rate(1)  # 1 Hz
    
    BATTERY_LEVEL = None
    BATTERY_ISCHARGING = False
    TASK = 'none'
    starting = rospy.get_time()

    
    rospy.Subscriber('/hrisim/robot_battery', BatteryStatus, cb_battery)
    battey_pub = rospy.Publisher('/hrisim/otext_robot_info', OverlayText, queue_size=10)
    
    while not rospy.is_shutdown():
        
        # Overlay text
        msg = create_overlay_text()
        battey_pub.publish(msg)
                
        rate.sleep()