/*!*****************************************************************************
 *  \file    belief_manager_process.h
 *  \brief   Definition of all the classes used in the file
 *           belief_manager_process.cpp .
 *
 *  \author  Guillermo De Fermin
 *  \copyright Copyright 2016 Universidad Politecnica de Madrid (UPM)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************/

#ifndef BELIEF_MANAGER_PROCESS
#define BELIEF_MANAGER_PROCESS

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <belief_manager_msgs/QueryBelief.h>
#include <belief_manager_msgs/AddBelief.h>
#include <belief_manager_msgs/RemoveBelief.h>
#include <belief_manager_msgs/CheckBeliefFormat.h>
#include <belief_manager_msgs/ListOfBeliefs.h>
#include <aerostack_msgs/StringStamped.h>
#include <droneMsgsROS/BeliefStatement.h>
#include <belief_manager_msgs/GenerateID.h>
#include <belief_manager_msgs/QueryLastGeneratedID.h>
#include "belief_manager.h"
#include <yaml-cpp/yaml.h>
#include <map>
#include <vector>

#define DEFAULT_INCREMENT 1
#define DEFAULT_START 0
#define DEFAULT_DRONE_ID 1

const std::string GENERATE_SERVICE_NAME = "generate_id";
const std::string QUERY_SERVICE_NAME = "query_last_generated_id";
const std::string BELIEF_OBJECT_NAME = "last_id";
const std::string DEBUG_LOG_STRING = "Something went wrong reconstructing the ID from the belief memory, \
  either it is too big (too much IDs requested) or someone is injecting non-numeric values";

/*!***************************************************************************
 *  \class BeliefManagerProcess
 *  \brief This process collects events and simplifies them for other processes
 *         to use.
 *****************************************************************************/
class BeliefManagerProcess{
public:
  BeliefManagerProcess();
  ~BeliefManagerProcess();

  void ownSetUp();
  void ownStart();
  void ownStop();
  void ownRun();

private:
  ros::NodeHandle n;

  BeliefManager belief_manager;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string config_file; 
  std::string config_path;     

  /* Custom variables ID Generator */
  unsigned int start_from;
  unsigned int increment;
  unsigned int last_id;

  unsigned int actual_id;

  /* Information publishers */
  ros::ServiceServer execute_query_server;
  ros::ServiceServer add_beliefs_server;
  ros::ServiceServer remove_beliefs_server;
  ros::ServiceServer check_belief_format_server;
  ros::ServiceServer generate_event_server;
  ros::ServiceServer query_event_server;

  ros::Publisher all_beliefs_publisher;
  ros::Publisher emergency_event_publisher;

  std::map<std::string, std::pair<std::string,int>> predicate_semantics;
  std::map<std::string,std::vector<std::string>> emergency_events;     
  std::map<std::string, std::pair<std::string,int>>::iterator predicate_semantics_it;
  std::map<std::string,std::vector<std::string>>::iterator emergency_events_it;     

  bool executeQuery(belief_manager_msgs::QueryBelief::Request& req,
                    belief_manager_msgs::QueryBelief::Response& res);
  bool addBeliefs(belief_manager_msgs::AddBelief::Request& req,
                    belief_manager_msgs::AddBelief::Response& res);
  bool removeBeliefs(belief_manager_msgs::RemoveBelief::Request& req,
                    belief_manager_msgs::RemoveBelief::Response& res);
  bool checkBeliefFormat(belief_manager_msgs::CheckBeliefFormat::Request& req,
                         belief_manager_msgs::CheckBeliefFormat::Response& res);

  void commonBeliefCallback(const droneMsgsROS::BeliefStatement& msg);

  bool generateIDCallback(belief_manager_msgs::GenerateID::Request& req, belief_manager_msgs::GenerateID::Response& res);

  bool queryLastGeneratedIDCallback(belief_manager_msgs::QueryLastGeneratedID::Request& req, belief_manager_msgs::QueryLastGeneratedID::Response& res);

  int parseBeliefQuery(std::string substitutions);
  unsigned int getBeliefOrOwnID();

  bool readConfigs(std::string configFile);
};

#endif //BELIEF_MANAGER_PROCESS
