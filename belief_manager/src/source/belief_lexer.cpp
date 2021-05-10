#include "../include/belief_lexer.h"

BeliefLexer::BeliefLexer() {
  // Define transitions
  defineTransition(InitialState, InitialState, "\\s");
  defineTransition(InitialState, WordState, "[\\w\\-]");
  defineTransition(WordState, WordState, "[\\w\\.]");
  defineTransition(InitialState, EndState, "[\\(\\),$]");
  defineTransition(WordState, EndState, "[\\(\\),\\s\\?$]");
  defineTransition(InitialState, VarState, "\\?");
  defineTransition(VarState, VarState, "\\w");
  defineTransition(VarState, EndState, "[\\(\\),\\s\\?$]");
}

void BeliefLexer::defineTransition(State s1, State s2, std::string rgx) {
  transitions[{s1, s2}] = boost::regex(rgx);
}

std::pair<std::string, std::string> BeliefLexer::nextToken(std::string& text) {
  // Special case to return the 'end' token
  if(text == "" || text[text.size()-1] != '$')
    text.append("$");

  State state = InitialState;
  std::pair<std::string, std::string> token = std::make_pair("error","");
  while(state != EndState && state != ErrorState) {
    State previous_state = state;
    state = nextState(state, text[0]);
    semanticAction(previous_state, state, token, text);
  }

  if(state == ErrorState)
    return std::make_pair("error", std::string(1, text[0]));

  return token;
}

void BeliefLexer::semanticAction(State s1, State s2, std::pair<std::string, std::string>& token, std::string& text) {
  if(s1 == InitialState && (s2 == VarState || s2 == WordState)) {
    token.first = (s2 == VarState)? "var": "word";
    token.second = "";
  }
  if(s2 == VarState || s2 == WordState) {
    token.second = token.second.append(std::string(1, text[0]));
  }
  if(s1 == InitialState && s2 == EndState) {
    token.first = std::string(1, text[0]);
    token.second = "";
  }
  if(!(s1 == WordState && s2 == EndState) && !(s1 == VarState && s2 == EndState) && s2 != ErrorState) {
    text.erase(text.begin());
  }
}

BeliefLexer::State BeliefLexer::nextState(State current_state, char c) {
  std::string next_char = std::string(1, c);
  State next_state = ErrorState;
  for(auto tr: transitions) {
    if(tr.first.first == current_state && boost::regex_match(next_char, tr.second))
      next_state = tr.first.second;
  }

  return next_state;
}
