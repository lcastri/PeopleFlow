/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <ros/ros.h>
#include <pedsim_srvs/GetNextDestination.h>  // Include the service message header

#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/agentgroup.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/waypoint.h>
#include <pedsim_simulator/force/force.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/waypointplanner.h>

Agent::Agent() {
  // initialize
  Ped::Tagent::setType(Ped::Tagent::ADULT);
  Ped::Tagent::setForceFactorObstacle(CONFIG.forceObstacle);
  forceSigmaObstacle = CONFIG.sigmaObstacle;
  Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial);

  ros::service::waitForService("get_next_destination");
  ros::NodeHandle nh;
  destination_client = nh.serviceClient<pedsim_srvs::GetNextDestination>("get_next_destination");

  // waypoints
  currentDestination = nullptr;
  nextDestination = nullptr;
  waypointplanner = nullptr;
  // state machine
  stateMachine = new AgentStateMachine(this);
  // group
  group = nullptr;
  task_duration = 0;
  initialTime_task = 0;
  initialTime_dest = 0;
  isPerformingTask = false;
  is_stuck = false;
}

Agent::~Agent() {
  // clean up
  foreach (Force* currentForce, forces) { delete currentForce; }
}

/// Calculates the desired force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::desiredForce() {
  Ped::Tvector force;
  if (!disabledForces.contains("Desired")) force = Tagent::desiredForce();

  // inform users
  emit desiredForceChanged(force.x, force.y);

  return force;
}

/// Calculates the social force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::socialForce() const {
  Ped::Tvector force;
  if (!disabledForces.contains("Social")) force = Tagent::socialForce();

  // inform users
  emit socialForceChanged(force.x, force.y);

  return force;
}

/// Calculates the obstacle force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::obstacleForce() const {
  Ped::Tvector force;
  if (!disabledForces.contains("Obstacle")) force = Tagent::obstacleForce();

  // inform users
  emit obstacleForceChanged(force.x, force.y);

  return force;
}

Ped::Tvector Agent::myForce(Ped::Tvector desired) const {
  // run additional forces
  Ped::Tvector forceValue;
  foreach (Force* force, forces) {
    // skip disabled forces
    if (disabledForces.contains(force->getName())) {
      // update graphical representation
      emit additionalForceChanged(force->getName(), 0, 0);
      continue;
    }

    // add force to the total force
    Ped::Tvector currentForce = force->getForce(desired);
    // → sanity checks
    if (!currentForce.isValid()) {
      ROS_DEBUG("Invalid Force: %s", force->getName().toStdString().c_str());
      currentForce = Ped::Tvector();
    }
    forceValue += currentForce;

    // update graphical representation
    emit additionalForceChanged(force->getName(), currentForce.x,
                                currentForce.y);
  }

  // inform users
  emit myForceChanged(forceValue.x, forceValue.y);

  return forceValue;
}

Ped::Twaypoint* Agent::getCurrentDestination() const {
  return currentDestination;
}

Ped::Twaypoint* Agent::updateDestination() {
  ros::Time now = ros::Time::now();
  double currentTimeSec = now.toSec();

  const int MAX_RETRIES = 3;  // Maximum number of retries
  int retry_count = 0;        // Keep track of retries
  bool request_successful = false;  // Track success of the request

  if (!is_stuck){
    if (task_duration != 0 && initialTime_task == 0) {
      initialTime_task = currentTimeSec;
    }
    if (!destinations.isEmpty() && currentTimeSec > (initialTime_task + task_duration)) {
      isPerformingTask = false;
      initialTime_task = 0;
      task_duration = 0;
      if ((!isInGroup()) || (isInGroup() && nextDestination == nullptr)) {
        // Request the next destination via service
        pedsim_srvs::GetNextDestination srv;
        // ROS_INFO("Agent %d requests a new destination", getId());
        
        // Build a list of waypoint names
        std::vector<std::string> waypoint_names;
        for (Waypoint* waypoint : destinations) {
            waypoint_names.push_back(waypoint->getName().toStdString());
        }
        srv.request.agent_id = getId();
        srv.request.origin.x = getx(); 
        srv.request.origin.y = gety(); 
        srv.request.origin.z = 0.0;
        srv.request.destinations = waypoint_names;
        srv.request.is_stuck = is_stuck;

        // Retry mechanism for the service call
        while (retry_count < MAX_RETRIES && !request_successful) {
          if (destination_client.call(srv)) {
            request_successful = true;  // Service call was successful

            task_duration = srv.response.task_duration;
            std::string dest_name = srv.response.destination_id;
            initialTime_dest = currentTimeSec;

            if (doesDestinationExist(dest_name)) {
              currentDestination = getDestinationByName(dest_name);
            } else {
              // Create new destination
              QString id = QString::fromStdString(dest_name);
              AreaWaypoint* w = new AreaWaypoint(id, srv.response.destination.x, srv.response.destination.y, srv.response.destination_radius);
              w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
              Waypoint* waypointPtr = dynamic_cast<Waypoint*>(w);
              currentDestination = waypointPtr;
            }

          } else {
            retry_count++;  // Increment retry count if service call failed
            ROS_WARN("Failed to call service GetNextDestination. Retrying %d/%d", retry_count, MAX_RETRIES);

            ros::Duration(1.0).sleep();  // Optional: add delay between retries
          }
        }

        // If all retries failed
        if (!request_successful) {
          ROS_ERROR("Failed to get a new destination after %d retries. Setting current position as destination.", MAX_RETRIES);
          QString id = QString::fromStdString("DEFAULT_" + std::to_string(getId()));
          AreaWaypoint* w = new AreaWaypoint(id, getx(), gety(), 0.0);  // Set radius to 0.0 to indicate it's the current position
          w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));  // Set the behavior (you can adjust this based on your needs)
          Waypoint* waypointPtr = dynamic_cast<Waypoint*>(w);
          currentDestination = waypointPtr;
        }

        if (isInGroup()) {
          AgentGroup* group = getGroup();
          QList<Agent*> group_members = group->getMembers();
          for (Agent* group_member : group_members) {
            if (group_member->getId() != getId()){
              group_member->nextDestination = currentDestination;
              // ROS_WARN("Agent %d updates the nextDestination of agent %d", getId(), group_member->getId());
            }
          }
        }
      } else if (isInGroup() && nextDestination != nullptr) {
        // ROS_WARN("Agent %d leverages the nextDestination", getId());
        currentDestination = nextDestination;
        nextDestination = nullptr;
      }
    } else if (!destinations.isEmpty() && currentTimeSec <= (initialTime_task + task_duration)) {
      isPerformingTask = true;
    }
  } else {
    // Request the next destination via service
    pedsim_srvs::GetNextDestination srv;
    ROS_WARN("Agent %d is stuck and resquest a new destination", getId());
    
    // Build a list of waypoint names
    std::vector<std::string> waypoint_names;
    for (Waypoint* waypoint : destinations) {
        waypoint_names.push_back(waypoint->getName().toStdString());
    }
    srv.request.agent_id = getId();
    srv.request.origin.x = getx(); 
    srv.request.origin.y = gety(); 
    srv.request.origin.z = 0.0;
    srv.request.destinations = waypoint_names;
    srv.request.is_stuck = is_stuck;

    // Retry mechanism for the service call
    while (retry_count < MAX_RETRIES && !request_successful) {
      if (destination_client.call(srv)) {
        request_successful = true;  // Service call was successful

        task_duration = srv.response.task_duration;
        std::string dest_name = srv.response.destination_id;
        initialTime_dest = currentTimeSec;

        if (doesDestinationExist(dest_name)) {
          currentDestination = getDestinationByName(dest_name);
        } else {
          // Create new destination
          QString id = QString::fromStdString(dest_name);
          AreaWaypoint* w = new AreaWaypoint(id, srv.response.destination.x, srv.response.destination.y, srv.response.destination_radius);
          w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
          Waypoint* waypointPtr = dynamic_cast<Waypoint*>(w);
          currentDestination = waypointPtr;
        }

      } else {
        retry_count++;  // Increment retry count if service call failed
        ROS_WARN("Failed to call service GetNextDestination. Retrying %d/%d", retry_count, MAX_RETRIES);
        ros::Duration(1.0).sleep();  // Optional: add delay between retries
      }
    }

    // If all retries failed
    if (!request_successful) {
      ROS_ERROR("Failed to get a new destination after %d retries. Setting current position as destination.", MAX_RETRIES);
      QString id = QString::fromStdString("DEFAULT_" + std::to_string(getId()));
      AreaWaypoint* w = new AreaWaypoint(id, getx(), gety(), 0.0);  // Set radius to 0.0 to indicate it's the current position
      w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));  // Set the behavior (you can adjust this based on your needs)
      Waypoint* waypointPtr = dynamic_cast<Waypoint*>(w);
      currentDestination = waypointPtr;
      is_stuck = true;
    } else {
      is_stuck = false;
    }
  }


  ros::Time returnTime = ros::Time::now();
  double returnTimeSec = now.toSec();
  // ROS_WARN("Agent %d Request time: %.10fs", getId(), returnTimeSec-currentTimeSec);

  return currentDestination;
}

bool Agent::doesDestinationExist(const std::string& name) {
  for (Waypoint* waypoint : destinations) {
    if (waypoint->getName().toStdString() == name) {
      return true;
    }
  }
  return false;
}

Waypoint* Agent::getDestinationByName(const std::string& name) {
  for (Waypoint* waypoint : destinations) {
    if (waypoint->getName().toStdString() == name) {
      return waypoint;
    }
  }
  return nullptr;
}

void Agent::updateState() {
  // check state
  stateMachine->doStateTransition();
}

void Agent::move(double h) {
  if (getType() == Ped::Tagent::ROBOT) {
    Ped::Tagent::SetRadius(CONFIG.robot_radius); // added by Luca
    if (CONFIG.robot_mode == RobotMode::TELEOPERATION) {
      // NOTE: Moving is now done by setting x, y position directly in
      // simulator.cpp
      // Robot's vx, vy will still be set for the social force model to work
      // properly wrt. other agents.

      // FIXME: This is a very hacky way of making the robot "move" (=update
      // position in hash tree) without actually moving it
      const double vx = getvx();
      const double vy = getvy();

      setvx(0);
      setvy(0);
      Ped::Tagent::move(h);
      setvx(vx);
      setvy(vy);
    } else if (CONFIG.robot_mode == RobotMode::CONTROLLED) {
      if (SCENE.getTime() >= CONFIG.robot_wait_time) {
        Ped::Tagent::move(h);
      }
    } else if (CONFIG.robot_mode == RobotMode::SOCIAL_DRIVE) {
      Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial * 0.7);
      Ped::Tagent::setForceFactorObstacle(35);
      Ped::Tagent::setForceFactorDesired(4.2);

      Ped::Tagent::setVmax(1.6);
      //Ped::Tagent::SetRadius(0.4); // added by Luca
      Ped::Tagent::move(h);
    }
  } else {
    Ped::Tagent::move(h);
  }

  if (getType() == Ped::Tagent::ELDER) {
    // Old people slow!
    Ped::Tagent::setVmax(0.9);
    Ped::Tagent::setForceFactorDesired(0.5);
  }

  // inform users
  emit positionChanged(getx(), gety());
  emit velocityChanged(getvx(), getvy());
  emit accelerationChanged(getax(), getay());
}

const QList<Waypoint*>& Agent::getWaypoints() const { return destinations; }

bool Agent::setWaypoints(const QList<Waypoint*>& waypointsIn) {
  destinations = waypointsIn;
  return true;
}

bool Agent::addWaypoint(Waypoint* waypointIn) {
  destinations.append(waypointIn);
  return true;
}

bool Agent::removeWaypoint(Waypoint* waypointIn) {
  const int removeCount = destinations.removeAll(waypointIn);

  return (removeCount > 0);
}

bool Agent::needNewDestination() const {
  if (waypointplanner == nullptr)
    return (!destinations.isEmpty());
  else {
    // ask waypoint planner
    return waypointplanner->hasCompletedDestination();
  }
}

bool Agent::isStuck() {
  if (waypointplanner == nullptr)
    is_stuck = false;
  else {
    ros::Time now = ros::Time::now();
    double currentTimeSec = now.toSec();
    if (!waypointplanner->hasCompletedDestination() && currentTimeSec > (initialTime_dest + task_duration + CONFIG.isStuckThreshold)) {
      ROS_WARN("Agent %d is stuck", getId());
      is_stuck = true;
    }
  }
  return is_stuck;
}

Ped::Twaypoint* Agent::getCurrentWaypoint() const {
  // sanity checks
  if (waypointplanner == nullptr) return nullptr;

  // ask waypoint planner
  return waypointplanner->getCurrentWaypoint();
}

bool Agent::isInGroup() const { return (group != nullptr); }

AgentGroup* Agent::getGroup() const { return group; }

void Agent::setGroup(AgentGroup* groupIn) { group = groupIn; }

bool Agent::addForce(Force* forceIn) {
  forces.append(forceIn);

  // inform users
  emit forceAdded(forceIn->getName());

  // report success
  return true;
}

bool Agent::removeForce(Force* forceIn) {
  int removeCount = forces.removeAll(forceIn);

  // inform users
  emit forceRemoved(forceIn->getName());

  // report success if a Behavior has been removed
  return (removeCount >= 1);
}

AgentStateMachine* Agent::getStateMachine() const { return stateMachine; }

WaypointPlanner* Agent::getWaypointPlanner() const { return waypointplanner; }

void Agent::setWaypointPlanner(WaypointPlanner* plannerIn) {
  waypointplanner = plannerIn;
}

QList<const Agent*> Agent::getNeighbors() const {
  // upcast neighbors
  QList<const Agent*> output;
  for (const Ped::Tagent* neighbor : neighbors) {
    const Agent* upNeighbor = dynamic_cast<const Agent*>(neighbor);
    if (upNeighbor != nullptr) output.append(upNeighbor);
  }

  return output;
}

void Agent::disableForce(const QString& forceNameIn) {
  // disable force by adding it to the list of disabled forces
  disabledForces.append(forceNameIn);
}

void Agent::enableAllForces() {
  // remove all forces from disabled list
  disabledForces.clear();
}

void Agent::setPosition(double xIn, double yIn) {
  // call super class' method
  Ped::Tagent::setPosition(xIn, yIn);

  // inform users
  emit positionChanged(xIn, yIn);
}

void Agent::setX(double xIn) { setPosition(xIn, gety()); }

void Agent::setY(double yIn) { setPosition(getx(), yIn); }

void Agent::setType(Ped::Tagent::AgentType typeIn) {
  // call super class' method
  Ped::Tagent::setType(typeIn);

  // inform users
  emit typeChanged(typeIn);
}

Ped::Tvector Agent::getDesiredDirection() const { return desiredforce; }

Ped::Tvector Agent::getWalkingDirection() const { return v; }

Ped::Tvector Agent::getSocialForce() const { return socialforce; }

Ped::Tvector Agent::getObstacleForce() const { return obstacleforce; }

Ped::Tvector Agent::getMyForce() const { return myforce; }

QPointF Agent::getVisiblePosition() const { return QPointF(getx(), gety()); }

void Agent::setVisiblePosition(const QPointF& positionIn) {
  // check and apply new position
  if (positionIn != getVisiblePosition())
    setPosition(positionIn.x(), positionIn.y());
}

QString Agent::toString() const {
  return tr("Agent %1 (@%2,%3)").arg(getId()).arg(getx()).arg(gety());
}
