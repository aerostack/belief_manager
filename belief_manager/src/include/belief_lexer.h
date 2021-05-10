/*!*****************************************************************************
 *  \file      belief_lexer.h
 *  \brief     Lexical analyzer to obtain tokens from belief string
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
#ifndef _BELIEF_LEXER_H
#define _BELIEF_LEXER_H

#include <unordered_map>
#include <utility>
#include <string>
#include <boost/regex.hpp>

class BeliefLexer {
public:
  BeliefLexer();

  /*!************************************************************************
   *  \brief  Obtains the next token from the given string, erasing it from
   *          the string.
   *  \return Pair which contains the token name as the first value and any
   *          additional information the token needs as the second value.
   *************************************************************************/
  std::pair<std::string, std::string> nextToken(std::string& text);


private:
  enum State {
    InitialState,
    WordState,
    VarState,
    EndState,
    ErrorState,
  };
  struct PairHash {
  public:
    std::size_t operator()(const std::pair<State, State> &x) const {
      return x.first*100 + x.second;
    }
  };

  std::unordered_map<std::pair<State, State>, boost::regex, PairHash> transitions;


  void defineTransition(State s1, State s2, std::string regex);
  State nextState(State current_state, char c);
  void semanticAction(State s1, State s2, std::pair<std::string, std::string>& token, std::string& text);
};
#endif
