#!/usr/bin/env python

from math import sqrt

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import Buffer, TransformListener
from robot_msgs.msg import BatteryStatus
import hrisim_util.ros_utils as ros_utils
from robot_srvs.srv import SetBattery, SetBatteryResponse

class SimBatteryManager():
    def __init__(self, init_level, static_duration, dynamic_duration, charging_time):
        self.lastT = None
        self.battery_level = init_level
        self.is_charging = False
        self.static_duration = static_duration
        self.dynamic_duration = dynamic_duration
        self.charging_time = charging_time
        
        self.K_nl_s = 100 / (self.static_duration * 3600)
        self.K_nl_d = (100 / (self.dynamic_duration * 3600) - self.K_nl_s)/(ROBOT_MAX_VEL)
        self.K_l_s = self.K_nl_s * OBS_FACTOR
        self.K_l_d = self.K_nl_d * OBS_FACTOR
        self.charge_rate = 100 / (self.charging_time * 3600)
        rospy.set_param("/robot_battery/noload_static_consumption", self.K_nl_s)
        rospy.set_param("/robot_battery/noload_dynamic_consumption", self.K_nl_d)
        rospy.set_param("/robot_battery/load_static_consumption", self.K_l_s)
        rospy.set_param("/robot_battery/load_dynamic_consumption", self.K_l_d)
        rospy.set_param("/robot_battery/charge_rate", self.charge_rate)

        self.vel = 0
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.cb_vel)
        rospy.Service('/hrisim/set_battery_level', SetBattery, self.set_battery_level_cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
         
    def cb_vel(self, odom):
        currentT = odom.header.stamp.to_sec()
        self.deltaT = currentT - self.lastT if self.lastT is not None else 0
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(), rospy.Duration(0.5))
            original_vector = Vector3Stamped()
            original_vector.header.frame_id = 'odom'
            original_vector.vector.x = odom.twist.twist.linear.x
            original_vector.vector.y = odom.twist.twist.linear.y
            original_vector.vector.z = odom.twist.twist.linear.z
            r_v = do_transform_vector3(original_vector, transform)
                      
            self.vel = sqrt(r_v.vector.x**2 + r_v.vector.y**2 + r_v.vector.z**2)
        except:
            self.vel = odom.twist.twist.linear.x
            
        self.is_charging = rospy.get_param("/robot_battery/is_charging")
        if not self.is_charging:
            self.discharge_battery()
        else:
            self.charge_battery()
            
        # Battery
        msg_Battery = BatteryStatus()
        msg_Battery.level.data = SBM.battery_level   
        msg_Battery.is_charging.data = SBM.is_charging        
        battey_pub.publish(msg_Battery)
        
        self.lastT = odom.header.stamp.to_sec()
    
    def discharge_battery(self):
        OBS = bool(ros_utils.wait_for_param('/hrisim/robot_obs'))
        KS = self.K_l_s if OBS else self.K_nl_s
        KD = self.K_l_d if OBS else self.K_nl_d
        self.battery_level -= self.deltaT * (KS + KD * self.vel)
        if self.battery_level < 0:
            self.battery_level = 0
        
    def charge_battery(self):
        # self.battery_level += np.floor(self.deltaT * (self.charge_rate))
        self.battery_level += self.deltaT * (self.charge_rate)
        if self.battery_level > 100:
            self.battery_level = 100  # Cap at 100%

    def set_battery_level_cb(self, req):
        if 0 <= req.battery_level <= 100:  # Ensure the level is valid
            self.battery_level = req.battery_level
            rospy.loginfo(f"Battery level set to {self.battery_level}")
            return SetBatteryResponse(success=True)
        else:
            rospy.logwarn("Invalid battery level. Must be between 0 and 100.")
            return SetBatteryResponse(success=False)
        
            
if __name__ == '__main__':
    rospy.init_node('robot_battery')
    rate = rospy.Rate(1)
    INIT_BATTERY = float(rospy.get_param("~init_battery", 100))
    STATIC_DURATION = float(rospy.get_param("~static_duration"))
    DYNAMIC_DURATION = float(rospy.get_param("~dynamic_duration"))
    OBS_FACTOR = float(rospy.get_param("~obs_factor"))
    CHARGING_TIME = float(rospy.get_param("~charging_time"))
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/TebLocalPlannerROS/max_vel_x"))

    SBM = SimBatteryManager(INIT_BATTERY, STATIC_DURATION, DYNAMIC_DURATION, CHARGING_TIME)
    
    battey_pub = rospy.Publisher('/hrisim/robot_battery', BatteryStatus, queue_size=10)
    rospy.set_param("/robot_battery/is_charging", False)
    
    rospy.spin()