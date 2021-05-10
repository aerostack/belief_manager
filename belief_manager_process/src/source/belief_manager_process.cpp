#include "belief_manager_process.h"

BeliefManagerProcess::BeliefManagerProcess() {
}

BeliefManagerProcess::~BeliefManagerProcess() {
}

void BeliefManagerProcess::ownSetUp() {
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("drone_id", drone_id, "1");

  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone"+drone_id);

  std::string int_arg;
  private_nh.param<std::string>("increment", int_arg, std::to_string(DEFAULT_INCREMENT));
  increment = std::stoi(int_arg);
  private_nh.param<std::string>("start_from", int_arg, std::to_string(DEFAULT_START));

  ros::param::get("~config_path", config_path);
  ros::param::get("~config_file", config_file);
  if ( config_file.length() == 0)
  {
      config_file="belief_manager_config.yaml";
  }
  std::cout<<"config_path="<<config_path<<std::endl;
  std::cout<<"config_file="<<config_file<<std::endl;

  bool readConfigsBool = readConfigs(config_path+"/"+config_file);
  if(!readConfigsBool){
      std::cout << "Error init"<< std::endl;
      return;
  }

  start_from = std::stoi(int_arg);
  actual_id=start_from;
  
}

void BeliefManagerProcess::ownStart() {
  execute_query_server = n.advertiseService("query_belief", &BeliefManagerProcess::executeQuery, this);
  add_beliefs_server = n.advertiseService("add_belief", &BeliefManagerProcess::addBeliefs, this);
  remove_beliefs_server = n.advertiseService("remove_belief", &BeliefManagerProcess::removeBeliefs, this);
  check_belief_format_server = n.advertiseService("check_belief_format", &BeliefManagerProcess::checkBeliefFormat, this);
  all_beliefs_publisher = n.advertise<belief_manager_msgs::ListOfBeliefs>("all_beliefs", 1000);
  emergency_event_publisher = n.advertise<aerostack_msgs::StringStamped>("/"+drone_id_namespace+"/emergency_event", 1000);
  generate_event_server = n.advertiseService(
    ros::this_node::getName() + "/" + GENERATE_SERVICE_NAME,
    &BeliefManagerProcess::generateIDCallback,
    this
  );
  query_event_server = n.advertiseService(
    ros::this_node::getName() + "/" + QUERY_SERVICE_NAME,
    &BeliefManagerProcess::queryLastGeneratedIDCallback,
    this
  );
}

bool BeliefManagerProcess::executeQuery(belief_manager_msgs::QueryBelief::Request& req,
                                        belief_manager_msgs::QueryBelief::Response& res) {
  BeliefManager::QueryResult result = belief_manager.executeQuery(req.query);
  res.success = result.success;
  res.substitutions = "";
  for(auto var: result.variables) {
    std::string var_name = var.first.substr(1, var.first.length());
    res.substitutions += var_name + ": " + var.second + "\n";
  }

  return true;
}

bool BeliefManagerProcess::addBeliefs(belief_manager_msgs::AddBelief::Request& req,
                                      belief_manager_msgs::AddBelief::Response& res) {
  bool success = belief_manager.addBeliefs(req.belief_expression, req.multivalued);
  res.success = success;
  belief_manager_msgs::ListOfBeliefs msg;
  msg.beliefs = belief_manager.getAllBeliefs();
  all_beliefs_publisher.publish(msg);
  //Check emergency event
  std::string belief = req.belief_expression;
  std::string belief_predicate = ""; 
  std::string belief_value = ""; 
  bool pred_bool = true;
  bool pred_value = false;
  for (auto x : belief){ 
   if (x == '(') pred_bool = false;
   if (x == ')') pred_value = false;
   if (pred_bool) belief_predicate = belief_predicate + x; 
   if (pred_value && x != ' ') belief_value = belief_value + x; 
   if (x == ',') pred_value = true;   
  }  
  std::cout<<belief<<std::endl;
  std::cout<<belief_predicate<<" , "<<belief_value<<std::endl;
  emergency_events_it = emergency_events.find(belief_predicate);
  if(emergency_events_it != emergency_events.end()){   
    for (int i= 0; i<emergency_events_it->second.size(); i++ ){
      if (emergency_events_it->second.at(i) == belief_value){
        aerostack_msgs::StringStamped urgent_msg;
        urgent_msg.header.stamp = ros::Time::now();
        urgent_msg.data = belief;
        emergency_event_publisher.publish(urgent_msg);
        std::cout<<"evergency_event detenceted";  
      }
    }
    //there is an emergency predicate without a value
    if(emergency_events_it->second.at(0)==""){
      belief = belief_predicate;
      std::cout<<belief<<std::endl;
      aerostack_msgs::StringStamped urgent_msg;
      urgent_msg.header.stamp = ros::Time::now();
      urgent_msg.data = belief;
      emergency_event_publisher.publish(urgent_msg);    
    }
  }
  return true;
}

bool BeliefManagerProcess::removeBeliefs(belief_manager_msgs::RemoveBelief::Request& req,
                                         belief_manager_msgs::RemoveBelief::Response& res) {
  bool success = belief_manager.removeBeliefs(req.belief_expression);
  res.success = success;

  belief_manager_msgs::ListOfBeliefs msg;
  msg.beliefs = belief_manager.getAllBeliefs();
  all_beliefs_publisher.publish(msg);

  return true;
}

bool BeliefManagerProcess::checkBeliefFormat(belief_manager_msgs::CheckBeliefFormat::Request& req,
                                         belief_manager_msgs::CheckBeliefFormat::Response& res) {
  bool success = belief_manager.checkBeliefFormat(req.belief_expression);
  res.success = success;

  return true;
}


int BeliefManagerProcess::parseBeliefQuery(std::string substitutions) {
  // string will be something like: "id: XXX \n",
  // if something went wrong "id: XXX, id: YYY \n",
  // if something went really wrong it won't be a number of be a number higher than the allowed
  // which is: 4294967295, corresponging with the maximum allowed for an unsigned int
  int ret = -1;
  int start_pos = substitutions.find(":") + 1;
  int end_pos = substitutions.find(",") - start_pos;
  std::string num_str = substitutions.substr(start_pos, end_pos);
  try{
    long int num = std::stol(num_str);
    // discard if out of range
    if( std::numeric_limits<unsigned int>::max() > num ){
      ret = (unsigned int) num;
    }else{
      ROS_DEBUG("%s", DEBUG_LOG_STRING);
    }
  }catch(const std::out_of_range& ex){
    ROS_DEBUG("%s", DEBUG_LOG_STRING);
  }catch(const std::invalid_argument& ex){
    ROS_DEBUG("%s", DEBUG_LOG_STRING);
  }
  return ret;
}

unsigned int BeliefManagerProcess::getBeliefOrOwnID() {

  return actual_id;
}

bool BeliefManagerProcess::generateIDCallback(belief_manager_msgs::GenerateID::Request& req,
                                            belief_manager_msgs::GenerateID::Response& res) {
  unsigned int next_id = getBeliefOrOwnID() + increment;
  // Update id from beliefs
  actual_id = next_id;

  res.id=actual_id;
  res.error_message ="";
  res.ack=true;
  return true;
}

bool BeliefManagerProcess::queryLastGeneratedIDCallback(belief_manager_msgs::QueryLastGeneratedID::Request& req,
                                                      belief_manager_msgs::QueryLastGeneratedID::Response& res) {
  res.id = getBeliefOrOwnID();
  res.error_message = "";
  res.ack = true;
  return true;
}

void BeliefManagerProcess::ownStop() {
  generate_event_server.shutdown();
  query_event_server.shutdown();
}

void BeliefManagerProcess::ownRun() {}

//Read
bool BeliefManagerProcess::readConfigs(std::string configFile){
    try{
      // Load file
      YAML::Node yamlconf = YAML::LoadFile(configFile);
      std::pair<std::string,int> pred;
      std::string p_name;

      if(yamlconf["predicate_semantics"].size() == 0){
        std::cerr<<"Error: predicate_semantics not found in belief_manager_config.yaml"<<std::endl;
        exit(-1);
      }

      for(int i = 0; i < yamlconf["predicate_semantics"].size(); i++){
        pred = std::make_pair (yamlconf["predicate_semantics"][i]["mutual_exclusive_values"].as<std::string>(),yamlconf["predicate_semantics"][i]["maximum_values"].as<int>());
        p_name = yamlconf["predicate_semantics"][i]["predicate_name"].as<std::string>();
        predicate_semantics.insert(std::make_pair(p_name, pred));
      }
      std::string aux;
      if(yamlconf["emergency_events"].size() == 0){
        std::cerr<<"Error: emergency_events not found in belief_manager_config.yaml"<<std::endl;
        exit(-1);
      }
      for(int i = 0; i < yamlconf["emergency_events"].size(); i++){
        if(!(yamlconf["emergency_events"][i]["emergency_value"])){
          aux="";
        }else{
          aux  = yamlconf["emergency_events"][i]["emergency_value"].as<std::string>();
          boost::algorithm::to_upper(aux);
        }
        p_name = yamlconf["emergency_events"][i]["predicate_name"].as<std::string>();


        emergency_events_it = emergency_events.find(p_name);
        if(emergency_events_it != emergency_events.end()){
          //Add new value to existent name
          emergency_events_it->second.push_back(aux);
        }else{
          //Insert new predicate name
          std::vector<std::string> vect_value;
          vect_value.push_back(aux);
          emergency_events.insert(std::make_pair(p_name,vect_value));
        }

      }    
      std::cout << "predicate_semantics:"<< std::endl;
      for(auto elem : predicate_semantics){
         std::cout << "   " << elem.first << ": (" << elem.second.first << ","  << elem.second.second <<")"<<"\n";
      }
      std::cout << "emergency_events:"<< std::endl;
      for(auto elem : emergency_events){
            for (int i = 0; i < elem.second.size(); ++i) std::cout << "   " << elem.first << ": " << elem.second.at(i) <<"\n";
      }

    }catch (std::exception& e){
       std::cerr<<(std::string("[YamlException! caller_function: Yaml file error ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n")<<std::endl;
       exit(-1);
    }
    return true;
}
