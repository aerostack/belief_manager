#include "../include/belief_manager.h"

BeliefManager::BeliefManager() {}

bool BeliefManager::checkBeliefFormat(std::string belief) {
  auto parsed_beliefs = parser.parse(belief);
  return parsed_beliefs.size() > 0 && parsed_beliefs[0].name != "$error$";
}

std::string BeliefManager::getAllBeliefs() {
  std::stringstream result;
  for(auto belief: belief_memory) {
    result << parser.toString(belief) << std::endl;
  }
  return result.str();
}

bool BeliefManager::addBeliefs(std::string beliefs, bool multivalued) {
  if(beliefs.find("?") != std::string::npos) {
    std::cout << "Error: beliefs to add must be grounded" << std::endl;
    return false;
  }

  auto parsed_beliefs = parser.parse(beliefs);

  bool success = parsed_beliefs.size() > 0 && parsed_beliefs[0].name != "$error$";
  if(!success && parsed_beliefs.size() > 1) {
    std::cout << parsed_beliefs[1].name << std::endl;
  }


  if(success) {
    for(auto belief: parsed_beliefs) {
      if(!multivalued) {
        // If belief isn't multivalued, erase the previous value (if any)
        for(int i=0; i<belief_memory.size(); i++) {
          auto b = belief_memory[i];
          if(b.name == belief.name && b.args.size() == belief.args.size()) {
            if(belief.args.size() == 1 && b.args[0] != belief.args[0]) {
              belief_memory.erase(belief_memory.begin()+i);
              break;
            } else if(belief.args.size() > 1 && b.args[0] == belief.args[0]) {
              belief_memory.erase(belief_memory.begin()+i);
              break;
            }
          }
        }
      }
      if(std::find(belief_memory.begin(), belief_memory.end(), belief) == belief_memory.end())
        belief_memory.push_back(belief);
    }
  }

  return success;
}

bool BeliefManager::removeBeliefs(std::string beliefs) {
  if(beliefs.find("?") != std::string::npos) {
    std::cout << "Error: beliefs to remove must be grounded" << std::endl;
    return false;
  }

  auto parsed_beliefs = parser.parse(beliefs);

  bool success = parsed_beliefs.size() > 0 && parsed_beliefs[0].name != "$error$";
  if(!success && parsed_beliefs.size() > 1) {
    std::cout << parsed_beliefs[1].name << std::endl;
  }

  if(success) {
    for(auto belief: parsed_beliefs) {
      if(std::find(belief_memory.begin(), belief_memory.end(), belief) == belief_memory.end())
        return false;
    }
    for(auto belief: parsed_beliefs) {
      belief_memory.erase(std::remove(belief_memory.begin(), belief_memory.end(), belief), belief_memory.end());
    }
  }

  return success;
}

bool BeliefManager::isMultivalued(BeliefParser::Predicate belief) {
  bool multivalued = false;

  for(int i=0; i<belief_memory.size(); i++) {
    auto b = belief_memory[i];
    if(b.name == belief.name && b.args.size() == belief.args.size()) {
      if(belief.args.size() == 1 && b.args[0] != belief.args[0]) {
        multivalued = true;
        break;
      } else if(belief.args.size() > 1 && b.args[0] == belief.args[0]) {
        multivalued = true;
        break;
      }
    }
  }

  return multivalued;
}
   
BeliefManager::QueryResult BeliefManager::executeQuery(std::string query) {
  std::vector<BeliefParser::Predicate> query_predicates = parser.parse(query);

  bool parse_result = query_predicates.size() > 0 && query_predicates[0].name != "$error$";
  if(!parse_result && query_predicates.size() > 1) {
    std::cout << query_predicates[1].name << std::endl;
  }
  if(!parse_result) {
    return {.success=false};
  }

  QueryResult result = {.success = false, .variables = {}};

  VarStack current_vars;
  QueryRestrictions current_restrictions;

  bool keep_going = true;
  while(keep_going) {
    keep_going = false;
    for(const auto& pred: query_predicates) {

      result.success = false;
      auto grounded_pred = groundPredicate(pred, current_vars.map);
      QueryResult query_result = partialQuery(grounded_pred, current_restrictions);


      if(query_result.success) {
        for(auto var: query_result.variables) {
          current_vars.push({var.first, var.second});
        }
      }

      if(!query_result.success)
        break;

      result.success = query_result.success;
    }
    if(!result.success) {
      if(!current_vars.empty()) {
        auto var = current_vars.top();
        if(result.variables.empty()) {
          if(current_vars.map.size() > 1) {
            current_vars.pop();
          }
        }
        current_restrictions.var = var.first;
        current_restrictions.values.push_back(var.second);
        keep_going = true;
        current_vars.pop();
      }
    }
  }

  if(result.success) {
    for(const auto& var: current_vars.map) {
      result.variables[var.first] = var.second;
    }
  }

  return result;
}

std::vector<BeliefManager::QueryResult> BeliefManager::executeMultiQuery(std::string query){
  // This supposes single-threaded access is used, no need for sync since we operate on the same thread
  std::vector<QueryResult> results;
  std::vector<std::pair<bool, std::string>> remove_queue;
  bool keep_going = true;
  while( keep_going ) {
    keep_going = false;
    QueryResult res = executeQuery(query);
    if( res.success ){
      keep_going = true;
      results.push_back(res);
      std::vector<BeliefParser::Predicate> predicates = parser.parse(query);
      bool parse_result = predicates.size() > 0 && predicates[0].name != "$error$";
      if( !parse_result ){
        // this should never happen, but just in case
        keep_going = false;
      }else{
        std::string to_remove = "";
        for(BeliefParser::Predicate pred : predicates) {
          BeliefParser::Predicate grounded = groundPredicate(pred, res.variables);
          bool multivalued = isMultivalued(grounded);
          std::string grounded_str = parser.toString(grounded);
          remove_queue.push_back({ multivalued, grounded_str });
          to_remove += grounded_str + ",";
        }
        // trailing ,
        to_remove.pop_back();
        removeBeliefs(to_remove);
      }
    }
  }
  for(const auto& to_push : remove_queue) {
    addBeliefs(to_push.second, to_push.first);
  }
  return results;
}

BeliefManager::QueryResult BeliefManager::partialQuery(BeliefParser::Predicate query, QueryRestrictions restrictions) {
  QueryResult result = {.success = false, .variables = {}};

  for(auto belief: belief_memory) {
    QueryResult unify_result = unify(query, belief);
    if(unify_result.success) {
      #define vectorContains(vec, val) (std::find(vec.begin(), vec.end(), val) != vec.end())
      if(vectorContains(restrictions.values, unify_result.variables[restrictions.var])) {
        // A restriction was used to unify
        continue;
      }
      #undef vectorContains

      // Pass on variables and success
      result.success = unify_result.success;
      for(auto var: unify_result.variables) {
        if(var.first != "" && var.second != "")
          result.variables[var.first] = var.second;
      }

      // No need to check rest of beliefs
      break;
    }
  }

  if(!result.success) {
    result.variables.clear();
  }
  return result;
}

BeliefManager::QueryResult BeliefManager::unify(BeliefParser::Predicate p1, BeliefParser::Predicate p2) {
  QueryResult result = {.success = true, .variables = {}};

  if(p1.name != p2.name) {
    if(p1.name.size() > 0 && p1.name[0] == '?') {
      if(p1.name.size() > 1) {
        result.variables[p1.name] = parser.toString(p2);
      }
      p1 = p2;
    } else if(p2.name.size() > 0 && p2.name[0] == '?') {
      if(p2.name.size() > 1) {
        result.variables[p2.name] = parser.toString(p1);
      }
      p2 = p1;
    } else {
      return {.success = false};
    }
  }

  if(p1.args == p2.args) {
    // Unified without changing any argument variables
    return result;
  }

  if(p1.args.size() != p2.args.size()) {
    return {.success = false};
  }

  for(int i=0; i<p1.args.size() && result.success; i++) {
    // First, ground known variables in predicates
    p1.args[i] = groundPredicate(p1.args[i], result.variables);
    p2.args[i] = groundPredicate(p2.args[i], result.variables);


    // Next, unify remaining
    QueryResult arg_result = unify(p1.args[i], p2.args[i]);

    // Pass on success and any new variables
    result.success = arg_result.success;
    for(auto var: arg_result.variables) {
      result.variables[var.first] = var.second;
    }
  }

  if(!result.success) {
    result.variables.clear();
  } else {
    fixVariables(result.variables);
  }

  return result;
}

BeliefParser::Predicate BeliefManager::groundPredicate(BeliefParser::Predicate pred, std::map<std::string, std::string> variables) {
  for(const auto& var: variables) {
    if(pred.name == var.first) {
      pred = parser.parse(var.second)[0];
    }
  }

  for(auto& arg: pred.args) {
    arg = groundPredicate(arg, variables);
  }

  return pred;
}

void BeliefManager::fixVariables(std::map<std::string, std::string>& variables) {
  auto vars_begin = variables;
  fixVariablesAux(variables);
  while(vars_begin != variables) {
    vars_begin = variables;
    fixVariablesAux(variables);
  }
}

void BeliefManager::fixVariablesAux(std::map<std::string, std::string>& variables) {
  for(auto& var: variables) {
    if(!var.second.empty() && var.second[0] == '?') {
      std::string value = variables[var.second];
      if(!value.empty() && value[0] != '?') {
        var.second = value;
      }
    }
  }
}


void BeliefManager::VarStack::pop() {
  auto e = stack.top();
  map.erase(e.first);
  stack.pop();
};

void BeliefManager::VarStack::push(const std::pair<std::string, std::string>& e) {
  stack.push(e);
  map[e.first] = e.second;
}

std::pair<std::string, std::string>& BeliefManager::VarStack::top() {
  return stack.top();
}

bool BeliefManager::VarStack::empty() {
  return stack.empty();
}

int BeliefManager::VarStack::size() {
  return map.size();
}
