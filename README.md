# ROS-Causal_HRISim
A Gazebo-based human-robot interaction simulator that accurately mimics HRI scenarios involving a [TIAGo](https://pal-robotics.com/robots/tiago/) robot and multiple pedestrians modelled using the [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) ROS library. 

## Features
* Customisable world
* Customisable people behaviours
* Customisable HRI scenario
* Customisable plans for the TIAGo robot
* Causal analysis

## How to use
### Build and run
After cloning the repository, use the following commands to build the Docker image and run it:
```
cd /path/to/ROS-Causal_HRISim
./HRISim_build.sh 
```
Once the Docker image is built, you can use the following command to run the container:
```
cd /path/to/ROS-Causal_HRISim
./HRISim_run.sh 
```
### Scenario setup and launch
Once inside the Docker container, run the following command to view the tmule file containing all the simulator parameters:
```
roscd hrisim_tmule/tmule
cat hrisim_bringup.yaml
```
Editable parameters:
* TIAGO_TYPE - represents the type of TIAGo;
* WORLD - world and map to load. It can be chosen among ["warehouse"]<br>
If you want to add your own WORLD, you can include your .world file in hrisim_gazebo/worlds and your map in hrisim_gazebo/tiago_maps.<br>
Note that the map must have the same name as the .world file;
* SCENARIO - pedsim scenario to load. It can be chosen among ["warehouse"]<br>
If you want to add your own SCENARIO, you can include your .xml file in /pedsim_ros/pedsim_simulator/scenarios;
* MAX_HUMAN_SPEED - teleop humam max speed
* ROBOT_RADIUS - robot size
* HUMAN_x - teleop human init x-coordinate
* HUMAN_y - teleop human init y-coordinate
* SPAWN_AGENT - bit to decide whether to spawn agents driven by social forces (if present in the SCENARIO) 
* SPAWN_TELEOP_AGENT - bit to decide whether to spawn teleop agent
* SPAWN_TIMEOUT - spawn timeout
* ALLOW_TASK - enable the performance of tasks when pedestrians reach their target position;
* MAX_TASKTIME - maximum task time;
* SCALING_FACTOR - time scaling factor. For example: SCALING_FACTOR = 12 ==> 1h corresponds to 5 mins;
* IS_STUCK_THRESHOLD - time threshold to decide whether a pedestrian is stuck;


If you want to modify any of these parameters, you can edit the hrisim_bringup.yaml file by:
```
roscd hrisim_tmule/tmule
nano hrisim_bringup.yaml
```
Once the tmule file is configured, you can start the simulator with the following command:
```
tstart
```
to visualise the tmule session
```
tshow
```
once inside the tmule, run the following command to change panel:
```
Ctrl+b
panel number [0-5]
```
and finally to stop it
```
Ctrl+b
panel number 0
tstop
```

### Planning
The ROS-Causal_HRISim includes the [PetriNetPlans](https://github.com/francescodelduchetto/PetriNetPlans) to define predefined plans for the TIAGo robot. The plan is a combination of actions and conditions that can be defined to create your own plan. Three different folders have been pre-created for plans, actions, and conditions, and they are:
* hrisim_plans
* hrisim_actions
* hrisim_conditions

For more details on how to define plans, actions, and conditions, visit the [PetriNetPlans](https://github.com/francescodelduchetto/PetriNetPlans) GitHub reposity.


## Recent changes
| Version | Changes |
| :---: | ----------- |
| 1.0.0 | package released|
