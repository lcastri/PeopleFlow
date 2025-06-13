#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import hrisim_util.ros_utils as ros_utils
from robot_srvs.srv import VisualisePath


def publish_waypoint_markers():
        marker_array = MarkerArray()

        for i, (wp_id, data) in enumerate(waypoints.items()):
            x = data['x']
            y = data['y']
            r = data['r']

            # CYLINDER marker (circle)
            circle_marker = Marker()
            circle_marker.header.frame_id = "map"
            circle_marker.header.stamp = rospy.Time.now()
            circle_marker.ns = "waypoints"
            circle_marker.id = i * 2  # Unique ID per marker
            circle_marker.type = Marker.CYLINDER
            circle_marker.action = Marker.ADD
            circle_marker.pose.position.x = x
            circle_marker.pose.position.y = y
            circle_marker.pose.position.z = 0.0
            circle_marker.pose.orientation.w = 1.0
            circle_marker.scale.x = r * 2
            circle_marker.scale.y = r * 2
            circle_marker.scale.z = 0.01
            circle_marker.color.r = 0.8
            circle_marker.color.g = 0.8
            circle_marker.color.b = 0.0
            circle_marker.color.a = 1.0
            circle_marker.lifetime = rospy.Duration()
            marker_array.markers.append(circle_marker)
            
            # TEXT marker (label)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "waypoints"
            text_marker.id = i * 2 + 1  # Different ID from circle
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.1  # Slightly above ground
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.5  # Font size
            text_marker.color.r = 0.1
            text_marker.color.g = 0.1
            text_marker.color.b = 0.1
            text_marker.color.a = 1.0
            text_marker.text = wp_id
            text_marker.lifetime = rospy.Duration()
            marker_array.markers.append(text_marker)

            pub.publish(marker_array)



if __name__ == '__main__':
    rospy.init_node('graph_visualiser')
    pub = rospy.Publisher('/hrisim/waypoints/markers', MarkerArray, queue_size=10)
    rate = rospy.Rate(5)

    waypoints = ros_utils.wait_for_param("/hrisim/wps")
    
    # Create a handle for the Trigger service
    rospy.sleep(5)
    ros_utils.wait_for_service('/graph/path/show')
    graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
    graph_path_show("")
    
    while not rospy.is_shutdown():
        publish_waypoint_markers()
        rate.sleep()
    