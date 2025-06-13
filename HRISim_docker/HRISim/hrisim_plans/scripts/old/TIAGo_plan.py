import os
import random
import sys
import xml.etree.ElementTree as ET
import rospy
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
from robot_msgs.msg import BatteryStatus
from move_base_msgs.msg import MoveBaseAction
import actionlib
import hrisim_util.ros_utils as ros_utils

DP = None
SHELFS_NAME = ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5", "shelf6"]
KITCHEN_NAME = ["table2", "table3", "table4", "table5", "table6", "kitchen1", "kitchen2", "kitchen3"]
SHELFS = {}
KITCHEN = {}
CHARGING_STATION = None
CLEANING_PATH = [
    (-18.22, 7.76), (-8.33, 7.90), (-8.03, -8.26), (-17.90, -8.16), (-17.89, -4.56), (-9.41, -4.44), (-9.36, -1.69), (-17.9, -1.5), (-17.9, 1.6), (-9.21, 1.6), (-9.21, 4.5), (-17.9, 4.5), (-18.22, 7.76),
    (-6.33, 8.73), (-6.33, -8.73), (-2.5, -8.73), (-2.5, 7.84), (9, 7.84), (9, 9), (-6.33, 8.73),
    (-0.83, 6.21), (1.3, 6.21), (1.3, 4.8), (-2.7, 4.8),
    (-0.92, 3.3), (1.3, 3.3), (1.3, 1.7), (-2.7, 1.7),
    (-0.875, 0.18), (1.18, 0.18), (1.18, -1.21), (-2.7, -1.21),
    (-0.92, -2.87), (1.35, -2.87), (1.35, -4.33), (-2.7, -4.33),
    (-2.3, -6.57), (3.3, -6.57), (3, 6), (9, 6), (5, 3), (9, 3), (9, 1), (5, 1), (5, -1.5), (9, -1.5), (9, -3.5), (5, -3.5), (5, -6.5), (9, -6.5), (9, -9), (-2.3, -9), (-2.3, -6.57)
    ]

def readScenario():
    # Load and parse the XML file
    tree = ET.parse(SCENARIO + '.xml')
    root = tree.getroot()
        
    # Parse schedule
    for wp in root.findall('waypoint'):
        if wp.get('id') in SHELFS_NAME:
            SHELFS[wp.get('id')] = [wp.get('x'), wp.get('y'), 0]
        elif wp.get('id') in KITCHEN_NAME:
            KITCHEN[wp.get('id')] = [wp.get('x'), wp.get('y'), 0]
        elif wp.get('id') == 'delivery_point':
            DP = [wp.get('x'), wp.get('y'), 0]
        elif wp.get('id') == 'charging_station':
            CHARGING_STATION = [wp.get('x'), wp.get('y'), 0]
    return SHELFS, DP, KITCHEN, CHARGING_STATION


def ac_goto(p, dest):
    p.action_cmd('goto', "_".join([str(coord) for coord in dest]), 'start')
    while not rospy.get_param('/hri/robot_busy'): 
        rospy.sleep(0.1)


def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global wp
    ros_utils.wait_for_param("/peopleflow/timeday")
    rospy.set_param('/hri/robot_busy', False)
    gotoDP = False
    rospy.set_param("/peopleflow/robot_plan_on", True)
    while True:
        
        if not rospy.get_param('/robot_battery/is_charging') and BATTERY_LEVEL <= 20:
            rospy.logwarn("Cancelling all goals..")
            client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            client.wait_for_server()
            client.cancel_all_goals()
            p.action_cmd('goto', "", 'interrupt')
                
            while rospy.get_param('/hri/robot_busy'): 
                rospy.sleep(0.1)
                                
            p.exec_action('goto', '_'.join([str(coord) for coord in CHARGING_STATION]))
                
            while rospy.get_param('/hri/robot_busy'): 
                rospy.sleep(0.1)
                
            rospy.set_param('/robot_battery/is_charging', True)
            TASK = "charging"
            rospy.logwarn("Battery charging..")
            
        elif BATTERY_LEVEL == 100 and rospy.get_param('/robot_battery/is_charging'):
            rospy.logwarn("Battery fully charged..")
            rospy.set_param('/robot_battery/is_charging', False)
        
        elif not rospy.get_param('/robot_battery/is_charging') and not rospy.get_param('/hri/robot_busy'):                        
            p.action_cmd('goto', "", 'interrupt')               
            # rospy.sleep(random.randint(10, 30))
                
            if rospy.get_param('/peopleflow/timeday') in ['starting', 'morning', 'lunch']:
                # Pick and Place
                TASK = "delivery"
                if gotoDP:
                    DEST = DP
                    gotoDP = False
                        
                else:
                    gotoDP = True
                    DEST = SHELFS[random.choice(SHELFS_NAME)]
                    
            elif rospy.get_param('/peopleflow/timeday') in ['afternoon', 'quitting']:
                # Inventory
                TASK = "inventory"
                DEST = SHELFS[random.choice(SHELFS_NAME)]
                    
            elif rospy.get_param('/peopleflow/timeday') in ['off']:
                # Cleaner
                TASK = "cleaning"
                if wp + 1 < len(CLEANING_PATH):
                    wp = wp + 1
                else:
                    wp = 0
                    break
                DEST = (CLEANING_PATH[wp][0], CLEANING_PATH[wp][1], 0)
            ac_goto(p, DEST)
        rospy.set_param('/hrisim/robot_task', TASK)
        
    rospy.set_param("/peopleflow/robot_plan_on", False) # triggers the ROS shutdown in ScenarioManager.py

                        
                        
def cb_battery(msg):
    global BATTERY_LEVEL, BATTERY_ISCHARGING
    BATTERY_LEVEL = float(msg.level.data)
    BATTERY_ISCHARGING = bool(msg.is_charging.data)
    
if __name__ == "__main__":  
    wp = -1
    SCENARIO = '/root/ros_ws/src/pedsim_ros/pedsim_simulator/scenarios/warehouse'
    SHELFS, DP, KITCHEN, CHARGING_STATION = readScenario()
    BATTERY_LEVEL = None
    BATTERY_ISCHARGING = None
    
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)

    p = PNPCmd()

    p.begin()

    Plan(p)

    p.end()