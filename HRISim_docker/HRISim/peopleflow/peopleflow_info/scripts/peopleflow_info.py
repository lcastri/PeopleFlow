#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
from peopleflow_msgs.msg import Time as pT

            
def cb_time(t: pT):
    
    text = OverlayText()
    text.width = 300  # Width of the overlay
    text.height = 25  # Height of the overlay
    text.left = 10  # X position (left offset)
    text.top = 10  # Y position (top offset)
    text.text_size = 13  # Font size
    text.line_width = 2
    text.text = f"{str(t.time_of_the_day.data).capitalize()}: {str(t.hhmmss.data)}"
    text.font = "DejaVu Sans Mono"
       
    # Text color
    text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
    
    overlaytime_pub.publish(text)
    
   
if __name__ == '__main__':
    rospy.init_node('peopleflow_info')
    rate = rospy.Rate(10)  # 10 Hz       
    
    overlaytime_pub = rospy.Publisher('/peopleflow/timeday', OverlayText, queue_size=10)
    rospy.Subscriber("/peopleflow/time", pT, cb_time)
                          

    rospy.spin()