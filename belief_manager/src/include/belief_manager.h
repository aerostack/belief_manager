/*!*****************************************************************************
 *  \file      belief_manager.h
 *  \brief     Syntax analyzer to parse beliefs string into a usable data
 *             structure
 *
 *  \author    Guillermo De Fermin
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
#ifndef _BELIEF_MANAGER_H
#define _BELIEF_MANAGER_H

#include <string>
#include <map>
#include <algorithm>
#include "../include/belief_parser.h"

class BeliefManager {
public:
  struct QueryResult {
    bool success;
    std::map<std::string, std::string> variables;
  };

  BeliefManager();

  bool checkBeliefFormat(std::string belief);
  bool addBeliefs(std::string beliefs, bool multivalued);
  bool removeBeliefs(std::string beliefs);
  QueryResult executeQuery(std::string query);
  std::vector<QueryResult> executeMultiQuery(std::string query);
  std::string getAllBeliefs();

private:
  struct QueryRestrictions {
    std::string var;
    std::vector<std::string> values;
    bool empty() {
      return var == "" || values.empty();
    };
  };
  struct VarStack {
    std::stack<std::pair<std::string, std::string>> stack;
    std::map<std::string, std::string> map;
    void pop();
    void push(const std::pair<std::string, std::string>& e);
    std::pair<std::string, std::string>& top();
    bool empty();
    int size();
  };

  BeliefParser parser;
  std::vector<BeliefParser::Predicate> belief_memory;

  bool isMultivalued(BeliefParser::Predicate belief);
  BeliefParser::Predicate groundPredicate(BeliefParser::Predicate pred, std::map<std::string, std::string> variables);
  void fixVariables(std::map<std::string, std::string>& variables);
  void fixVariablesAux(std::map<std::string, std::string>& variables);
  QueryResult unify(BeliefParser::Predicate p1, BeliefParser::Predicate p2);
  QueryResult partialQuery(BeliefParser::Predicate query, QueryRestrictions r);
};

#endif
