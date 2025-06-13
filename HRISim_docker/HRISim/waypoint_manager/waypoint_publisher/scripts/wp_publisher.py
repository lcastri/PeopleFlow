#!/usr/bin/env python

import rospy
import xml.etree.ElementTree as ET

def parse_waypoints(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    waypoints = {}

    for wp in root.findall('waypoint'):
        wp_id = wp.get('id')
        x = float(wp.get('x'))
        y = float(wp.get('y'))
        r = float(wp.get('r'))
        waypoints[wp_id] = {"x": x, "y": y, "r": r}
    
    return waypoints



if __name__ == "__main__":
    try:
        rospy.init_node('wp_publisher')

        xml_path = rospy.get_param("~xml_file")

        rospy.loginfo(f"Parsing waypoints from: {xml_path}")
        waypoints = parse_waypoints(xml_path)

        rospy.set_param('hrisim/wps', waypoints)
        rospy.loginfo(f"Published {len(waypoints)} waypoints to /hrisim/wps")
    except rospy.ROSInterruptException:
        pass
