#!/usr/bin/env python

from enum import Enum
import sys
import pandas as pd
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import WPPeopleCounters, Time as pT
from tiago_battery.msg import BatteryStatus
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf
from rosgraph_msgs.msg import Clock


NODE_NAME = 'hrisim_postprocess'
NODE_RATE = 10 #Hz

class Task(Enum):
    DELIVERY = "delivery"
    INVENTORY = "inventory"
    CLEANING = "cleaning"
    CHARGING = "charging"

TASKS = {
    Task.DELIVERY.value: 0,
    Task.INVENTORY.value: 1,
    Task.CLEANING.value: 2,
    Task.CHARGING.value: 3
}

class Robot():
    
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        self.gx = 0
        self.gy = 0
        self.task = -1
        self.battery_level = 0
        self.is_charging = 0
        
        
class Agent():
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        

def getPose(p: Pose):
    x = p.position.x
    y = p.position.y
    
    q = (
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w
    )
    
    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return x, y, yaw
        

class DataManager():
    """
    Class handling data
    """
    
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
        self.rostime = 0
        self.robot = Robot()
        self.agents = {}
        
        self.WPs = {}
        self.peopleAtWork = 0
        self.timeOfDay = ''        
        self.hhmmss = ''
        self.elapsed = 0

        # subscribers
        rospy.Subscriber("/clock" , Clock, self.cb_clock)
        rospy.Subscriber("/robot_pose" , PoseWithCovarianceStamped, self.cb_robot_pose)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.cb_odom)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_robot_goal)
        rospy.Subscriber("/hrisim/robot_task", String, self.cb_robot_task)
        rospy.Subscriber("/peopleflow/counter", WPPeopleCounters, self.cb_people_counter)
        rospy.Subscriber("/peopleflow/time", pT, self.cb_time)
        rospy.Subscriber("/pedsim_simulator/simulated_agents" , AgentStates, self.cb_agents)
        rospy.Subscriber("/hrisim/tiago_battery" , BatteryStatus, self.cb_robot_battery)
    
        
    def cb_clock(self, clock: Clock):
        self.rostime = clock.clock.to_sec()
        
        
    def cb_robot_pose(self, pose: PoseWithCovarianceStamped):
        self.rostime = pose.header.stamp.to_sec()
        self.robot.x, self.robot.y, self.robot.yaw = getPose(pose.pose.pose)
        
        
    def cb_odom(self, odom: Odometry):
        self.robot.v = abs(odom.twist.twist.linear.x)
        
        
    def cb_robot_goal(self, goal: MoveBaseActionGoal):
        self.robot.gx = goal.goal.target_pose.pose.position.x
        self.robot.gy = goal.goal.target_pose.pose.position.y
    
    
    def cb_robot_task(self, task: String):
        self.robot.task = TASKS[task.data]
        
        
    def cb_people_counter(self, wps: WPPeopleCounters):
        self.peopleAtWork = wps.numberOfWorkingPeople
        self.timeOfDay = wps.counters[0].time.data
        for wp in wps.counters:
            self.WPs[wp.WP_id.data] = wp.numberOfPeople
            
            
    def cb_time(self, t: pT):
        self.timeOfDay = t.time_of_the_day.data
        self.hhmmss = t.hhmmss.data
        self.elapsed = t.elapsed
        
        
    def cb_agents(self, people: AgentStates):        
        for person in people.agent_states:
            if person.id not in self.agents:
                self.agents[person.id] = Agent()
            self.agents[person.id].x, self.agents[person.id].y, self.agents[person.id].yaw = getPose(person.pose)
            self.agents[person.id].v = person.twist.linear.x      


    def cb_robot_battery(self, b: BatteryStatus):
        self.robot.battery_level = b.level.data
        self.robot.is_charging = b.is_charging.data
        

def save_data_to_csv(data_rows, filename, csv_path, wps, general_columns_name):
    df = pd.DataFrame(data_rows)
    df.to_csv(f"{csv_path}/{filename}.csv", index=False)
    rospy.logwarn(f"Saved {filename}.csv")

    # Save WP-specific DataFrames
    for wp_id, np in wps.items():
        wp_df = pd.DataFrame(
            [{key: row[key] for key in row if key in (general_columns_name + [wp_id])} for row in data_rows]
        )
        wp_filename = f"{filename}_{wp_id}.csv"
        wp_df.to_csv(f"{csv_path}/{wp_filename}", index=False)
        rospy.logwarn(f"Saved {wp_filename}")


def shutdown_callback(data_rows, bagname, current_time_of_day, csv_path, wps, general_columns_name):
    rospy.logwarn("Shutting down and saving data.")
    filename = f"{bagname}_{current_time_of_day}"
    save_data_to_csv(data_rows, filename, csv_path, wps, general_columns_name)


if __name__ == '__main__':
    BAGNAME = sys.argv[1]
    CSV_PATH = sys.argv[2]
    TIMEOFTHEDAY = sys.argv[3]

    # Init node
    rospy.init_node(NODE_NAME)

    # Set node rate
    rate = rospy.Rate(NODE_RATE)

    data_handler = DataManager()

    # Initialize variables to track timeOfDay
    current_time_of_day = None
    recording = False
    data_rows = []  # List to store collected data for each segment

    # Register shutdown callback
    rospy.on_shutdown(lambda: shutdown_callback(
        data_rows, BAGNAME, current_time_of_day, CSV_PATH, data_handler.WPs, GENERAL_COLUMNS_NAME))

    while not rospy.is_shutdown():
        # rospy.logwarn(data_handler.elapsed)
        if data_handler.timeOfDay == '':
            continue

        # Start recording when timeOfDay matches the specified time
        if not recording and data_handler.timeOfDay == TIMEOFTHEDAY:
            current_time_of_day = data_handler.timeOfDay
            rospy.logwarn(f"Started recording at {TIMEOFTHEDAY}")
            recording = True

        # Stop recording and trigger shutdown when timeOfDay changes
        if recording and data_handler.timeOfDay != TIMEOFTHEDAY:
            rospy.logwarn(f"Time of day changed from {TIMEOFTHEDAY}, triggering shutdown")
            rospy.signal_shutdown("Time of day changed")

        if recording:
            # Collect data for the current time step
            data_row = {
                'ros_time': data_handler.rostime,
                'pf_elapsed_time': data_handler.elapsed,
                'time_of_day': data_handler.timeOfDay,
                'hhmmss': data_handler.hhmmss,
                'r_x': data_handler.robot.x,
                'r_y': data_handler.robot.y,
                'r_yaw': data_handler.robot.yaw,
                'r_v': data_handler.robot.v,
                'g_x': data_handler.robot.gx,
                'g_y': data_handler.robot.gy,
                'r_task': data_handler.robot.task,
                'r_battery': data_handler.robot.battery_level,
                'is_charging': 1 if data_handler.robot.is_charging else 0,
                'people_at_work': data_handler.peopleAtWork,
            }

            # Collect agents' data
            for agent_id, agent in data_handler.agents.items():
                data_row[f'a{agent_id}_x'] = agent.x
                data_row[f'a{agent_id}_y'] = agent.y
                data_row[f'a{agent_id}_yaw'] = agent.yaw
                data_row[f'a{agent_id}_v'] = agent.v

            GENERAL_COLUMNS_NAME = list(data_row.keys())

            # Collect WP' data
            for wp_id, np in data_handler.WPs.items():
                data_row[f'{wp_id}'] = np

            # Append the row to the list
            data_rows.append(data_row)

        rate.sleep()