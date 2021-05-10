#include "../include/belief_manager.h"
#include "../include/belief_parser.h"
#include <iostream>

using namespace std;

// Compile by hand:
// g++ -std=c++14 source/*.cpp  test/belief_manager_multi_query_test.cpp -lboost_regex

string toString(BeliefManager::QueryResult result){
  stringstream res;
  if( result.success ){
    res<<"Len "<<result.variables.size()<<endl;
    for(auto var: result.variables) {
      res<<var.first + ": " + var.second + "\n";
    }
  }else{
    res<<"No success "<<endl;
  }
  return res.str();
}

void fetchSingleQuery(BeliefManager manager, string query) {
  BeliefManager::QueryResult result = manager.executeQuery(query);
  cout<<toString(result);
}

void fetchQuery(BeliefManager manager, string query){
  vector<BeliefManager::QueryResult> results = manager.executeMultiQuery(query);
  for(BeliefManager::QueryResult result: results){
    cout<<toString(result);
  }
}

int main(int argc, char** argv){

  BeliefManager manager;

  vector<pair<string, bool>> objects;
  objects.push_back(make_pair("qr_code", true));
  objects.push_back(make_pair("aruco", false));
  objects.push_back(make_pair("aruco", true));
  objects.push_back(make_pair("qr_code", true));
  objects.push_back(make_pair("aruco", true));
  objects.push_back(make_pair("qr_code", false));

  string object_beliefs = "";
  string code_beliefs = "";
  string visible_beliefs = "";

  int start_id = 350;
  int visible_added = 0;

  for(int i = 0; i < objects.size(); i++){
    object_beliefs += "object(" + to_string(i + start_id) + ", " + objects[i].first + "), ";
    code_beliefs += "code(" + to_string(i + start_id) + ", " + to_string(i) + "), ";
    if( objects[i].second ){
      visible_beliefs += "visible(" + to_string(i + start_id) + "), ";
    }
  }

  // remove trailing comma and space
  for(int i = 0; i < 2; i++){
    object_beliefs.pop_back();
    code_beliefs.pop_back();
    visible_beliefs.pop_back();
  }

  manager.addBeliefs(object_beliefs, false);
  manager.addBeliefs(code_beliefs, false);
  manager.addBeliefs(visible_beliefs, true);
  string query_str = "code(?x, ?y), object(?x, ?z)";
  fetchQuery(manager, query_str);

  cout<<"Everything:\n"<<manager.getAllBeliefs()<<endl;

  return 0;
}

