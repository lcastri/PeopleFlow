#!/usr/bin/env python

from math import sqrt
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import Buffer, TransformListener
from tiago_battery.msg import BatteryStatus
        
class SimBatteryManager():
    def __init__(self, init_level, static_duration, dynamic_duration, charging_time):
        self.battery_level = init_level
        self.is_charging = False
        self.static_duration = static_duration
        self.dynamic_duration = dynamic_duration
        self.charging_time = charging_time
        
        self.static_consumption = 100 / (self.static_duration*3600)
        self.K = (100 / (self.dynamic_duration*3600) - self.static_consumption)/0.25
        self.charge_rate = 100 / (self.charging_time * 3600)

        self.vel = 0
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.cb_vel)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
         
    def cb_vel(self, odom):
        transform = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(), rospy.Duration(0.5))
        original_vector = Vector3Stamped()
        original_vector.header.frame_id = 'odom'
        original_vector.vector.x = odom.twist.twist.linear.x
        original_vector.vector.y = odom.twist.twist.linear.y
        original_vector.vector.z = odom.twist.twist.linear.z
        r_v = do_transform_vector3(original_vector, transform)
                      
        self.vel = sqrt(r_v.vector.x**2 + r_v.vector.y**2 + r_v.vector.z**2)
    
    def discharge_battery(self):
        self.battery_level -= (self.static_consumption + self.K * self.vel**2) * SCALING_FACTOR
        if self.battery_level < 0:
            self.battery_level = 0
        
    def charge_battery(self):
        self.battery_level += (self.charge_rate) * SCALING_FACTOR
        if self.battery_level > 100:
            self.battery_level = 100  # Cap at 100%


def wait_for_param(param_name, timeout=30):
    rospy.loginfo(f"Waiting for parameter: {param_name}")
    while not rospy.has_param(param_name):
        rospy.sleep(0.1)
    return rospy.get_param(param_name)
    
if __name__ == '__main__':
    rospy.init_node('tiago_battery')
    rate = rospy.Rate(1)  # 1 Hz
    INIT_BATTERY = float(rospy.get_param("~init_battery", 100))
    STATIC_DURATION = float(rospy.get_param("~static_duration"))
    DYNAMIC_DURATION = float(rospy.get_param("~dynamic_duration"))
    CHARGING_TIME = float(rospy.get_param("~charging_time"))
    SCALING_FACTOR = float(wait_for_param("/peopleflow_manager/scaling_factor"))
    
    SBM = SimBatteryManager(INIT_BATTERY, STATIC_DURATION, DYNAMIC_DURATION, CHARGING_TIME)
    
    battey_pub = rospy.Publisher('/hrisim/tiago_battery', BatteryStatus, queue_size=10)
    rospy.set_param("/tiago_battery/is_charging", False)
    while not rospy.is_shutdown():
        SBM.is_charging = rospy.get_param("/tiago_battery/is_charging")
        if not SBM.is_charging:
            SBM.discharge_battery()
        else:
            SBM.charge_battery()
            
        # Battery
        msg_Battery = BatteryStatus()
        msg_Battery.level.data = SBM.battery_level        
        msg_Battery.is_charging.data = SBM.is_charging        
        battey_pub.publish(msg_Battery)
        
        rate.sleep()