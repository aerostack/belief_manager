#include "../include/belief_parser.h"

BeliefParser::BeliefParser() {
  initParserTable();
}

void BeliefParser::initParserTable() {
  parser_table[{"B", "word"}] = {"P", "Y"};
  parser_table[{"B", "var"}] = {"P", "Y"};
  parser_table[{"B", "("}] = {"P", "Y"};

  parser_table[{"Y", ","}] = {",", "B"};
  parser_table[{"Y", "$"}] = {"lambda"};

  parser_table[{"P", "word"}] = {"word", "Z"};
  parser_table[{"P", "var"}] = {"var", "Z"};
  parser_table[{"P", "("}] = {"(", "L", ")"};

  parser_table[{"Z", "("}] = {"(", "L", ")"};
  parser_table[{"Z", ")"}] = {"lambda"};
  parser_table[{"Z", ","}] = {"lambda"};
  parser_table[{"Z", "$"}] = {"lambda"};

  parser_table[{"L", "word"}] = {"P", "W"};
  parser_table[{"L", "var"}] = {"P", "W"};
  parser_table[{"L", "("}] = {"P", "W"};

  parser_table[{"W", ")"}] = {"lambda"};
  parser_table[{"W", ","}] = {",", "L"};
}

std::vector<BeliefParser::Predicate> BeliefParser::parse(std::string text) {
  std::vector<Predicate> predicates;
  std::stack<std::vector<Predicate>*> list_at;
  list_at.push(&predicates);

  auto token = lexer.nextToken(text);
  parser_stack.push("$");
  parser_stack.push("B");
  std::string top = parser_stack.top();

  while(true) {
    if(top == "$" && token.first == "$") {
      // Program interpreted successfully
      break;
    } else if(isTerminal(top)) {
      if(top == token.first) {
        // Recognized
        parser_stack.pop();
        recognizeToken(list_at, token);
        token = lexer.nextToken(text);
      } else {
        // TODO: error
        std::cout << "Error 1" << std::endl;
        break;
      }
    } else {
      auto table_entry = parser_table[{top, token.first}];
      if(!table_entry.empty()) {
        parser_stack.pop();
        for(auto entry: boost::adaptors::reverse(table_entry)) {
          if(entry != "lambda") {
            parser_stack.push(entry);
          }
        }
      } else {
        std::string readable_token = token.second.empty()? token.first: token.second;
        std::string tkn_word = token.first == "error"? "symbol": "token";
        return {{.name="$error$"}, {.name="Error: unexpected "+tkn_word+" '"+readable_token+"'"}};
      }
    }
    top = parser_stack.top();
  }

  if(!predicates.empty() &&
     predicates[predicates.size()-1].name == "" &&
     predicates[predicates.size()-1].args.empty()) {
    predicates.pop_back();
  }

  return predicates;
}

std::string BeliefParser::toString(Predicate p) {
  std::string result = p.name;
  if(!p.args.empty()) {
    result.append("(");
    for(auto arg: p.args) {
      result.append(toString(arg) + ", ");
    }
    // Remove trailing comma and space
    result.pop_back();
    result.pop_back();

    result.append(")");
  }

  return result;
}

void BeliefParser::recognizeToken(std::stack<std::vector<Predicate>*>& list_at,
                                  std::pair<std::string, std::string> token) {
  auto current_list = list_at.top();
  if(token.first == "word" || token.first == "var") {
    if((*current_list).empty()) {
      (*current_list).push_back({.name = token.second, .args = {}});
    } else {
      (*current_list)[(*current_list).size()-1] = {.name = token.second, .args = {}};
    }
  } else if(token.first == "(") {
    // Change to args list
    if((*current_list).empty()) {
      (*current_list).push_back({.name = "", .args = {}});
    }
    current_list = &(*current_list)[(*current_list).size()-1].args;
    list_at.push(current_list);
  } else if(token.first == ")") {
    // Go back to parent list
    list_at.pop();
  } else if(token.first == ",") {
    // Advance in list
    (*current_list).push_back({});
  }
}

bool BeliefParser::isTerminal(std::string elm) {
  return elm.size() && !std::isupper(elm[0]);
}

bool BeliefParser::Predicate::operator==(const Predicate& p) const {
  return (name == p.name) && ((args.empty() && p.args.empty()) || args == p.args);
}

bool BeliefParser::Predicate::operator!=(const Predicate& p) const {
  return !((*this) == p);
}

bool BeliefParser::Predicate::grounded() {
  if(!name.empty() && name[0] == '?') {
    return false;
  } else if(args.empty()) {
    return true;
  } else {
    for(auto arg: args) {
      if(!arg.grounded())
        return false;
    }
    return true;
  }
}
