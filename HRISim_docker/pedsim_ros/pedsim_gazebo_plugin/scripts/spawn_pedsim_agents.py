#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg  import AgentStates

# xml file containing a gazebo model to represent agent, currently is represented by a cubic but can be changed
global xml_file

def cb_actor_poses(actors):
    global AGENT_SPAWNED
    if not AGENT_SPAWNED:
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            rospy.loginfo("Spawning model: actor_id = %s", actor_id)

            # model_pose = Pose(Point(x= actor_pose.position.x - 0.731689,
            model_pose = Pose(Point(x= actor_pose.position.x,
                                    y= actor_pose.position.y + 0.002827,
                                    z= actor_pose.position.z + 0.75),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            spawn_model(actor_id, xml_string, "", model_pose, "world")
        rospy.logwarn("All autonomous agents have been spawn")
        AGENT_SPAWNED = True
        
def cb_teleop_actor_poses(actors):
    global TELEOP_AGENT_SPAWNED
    if not TELEOP_AGENT_SPAWNED:
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            rospy.loginfo("Spawning model: actor_id = %s", actor_id)

            model_pose = Pose(Point(x= actor_pose.position.x,
                                y= actor_pose.position.y,
                                z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            spawn_model(actor_id, xml_string, "", model_pose, "world")
        rospy.logwarn("All teleop agents have been spawn")
        TELEOP_AGENT_SPAWNED = True

if __name__ == '__main__':

    rospy.init_node("spawn_pedsim_agents")
    rate = rospy.Rate(10)
    
    global AGENT_SPAWNED, TELEOP_AGENT_SPAWNED
    AGENT_SPAWNED = not bool(rospy.get_param('/pedsim_simulator/spawn_agent'))
    TELEOP_AGENT_SPAWNED = not bool(rospy.get_param('/pedsim_simulator/spawn_teleop_agent'))
    TIMEOUT = float(rospy.get_param('/pedsim_simulator/spawn_timeout', 10))
    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
    default_actor_model_file = pkg_path + "/models/actor_model.sdf"

    actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
    file_xml = open(actor_model_file)
    xml_string = file_xml.read()

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    print("service: spawn_sdf_model is available ....")
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, cb_actor_poses)
    rospy.Subscriber("/ped/control/gz_persons", AgentStates, cb_teleop_actor_poses)

    init = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if init - rospy.Time.now().to_sec() >= TIMEOUT: rospy.signal_shutdown("Timeout")
        if AGENT_SPAWNED and TELEOP_AGENT_SPAWNED:
            rospy.signal_shutdown("All agents have been spawned!")
        rate.sleep()
