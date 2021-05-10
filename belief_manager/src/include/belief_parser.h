/*!*****************************************************************************
 *  \file      belief_parser.h
 *  \brief     Syntax analyzer to parse belief string into a usable data
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
#ifndef _BELIEF_PARSER_H
#define _BELIEF_PARSER_H

#include <string>
#include <vector>
#include <utility>
#include <map>
#include <stack>
#include <cctype>
#include <boost/range/adaptor/reversed.hpp>
#include "../include/belief_lexer.h"
#include <iostream>

class BeliefParser {
public:
  struct Predicate {
    std::string name;
    std::vector<Predicate> args;
    bool operator==(const Predicate& p) const;
    bool operator!=(const Predicate& p) const;
    bool grounded();
  };

  BeliefParser();

  std::vector<Predicate> parse(std::string text);
  std::string toString(Predicate);

private:
  BeliefLexer lexer;
  std::map<std::pair<std::string, std::string>, std::vector<std::string>> parser_table;
  std::stack<std::string> parser_stack;

  void initParserTable();
  bool isTerminal(std::string);
  void recognizeToken(std::stack<std::vector<Predicate>*>& list_at, std::pair<std::string, std::string> token);
};

#endif
