#include <gtest/gtest.h>
#include "belief_manager_process.h"
#include <iostream>
#include <vector>
using namespace std;
 
ros::NodeHandle* n;

void addBeliefs(std::string beliefs, bool multievalued) {
  aerostack_msgs::AddBelief srv;
  srv.request.belief_expression = beliefs;
  srv.request.multivalued = multievalued;
  while(!n->serviceClient<aerostack_msgs::AddBelief>("add_belief").call(srv)) {
    ros::Duration(1).sleep();
  }
}

void removeBeliefs(std::string beliefs) {
  aerostack_msgs::RemoveBelief srv;
  srv.request.belief_expression = beliefs;
  while(!n->serviceClient<aerostack_msgs::RemoveBelief>("remove_belief").call(srv)) {
    ros::Duration(1).sleep();
  }
}

void callback(const aerostack_msgs::ListOfBeliefs & msg)
{
  ASSERT_EQ(msg.beliefs,"");
}


void no_beliefs(){
  while(!n->subscribe("all_beliefs", 1000, &callback)){
    ros::Duration(1).sleep();
  }
}


aerostack_msgs::QueryBelief::Response query_belief(std::string query) {
  aerostack_msgs::QueryBelief srv;
  srv.request.query = query;
  while(!n->serviceClient<aerostack_msgs::QueryBelief>("query_belief").call(srv)) {
    ros::Duration(1).sleep();
  }
  return srv.response;
}



vector<string> getpairs(string subs){
vector<string> recortes;
   int ini=0;
   int pos=0;
    while((pos=subs.find("\n",pos))!=string::npos){
      recortes.push_back(subs.substr(ini,pos-ini));
      pos=pos+1;
      ini=pos;

  }


//now we are going to delete spaces
  vector<string> res;
  for(int j=0;j<recortes.size();j++){
      string aux="";
      for(int  i = 0; recortes[j][i] != 0;i++){
              if(recortes[j][i] != 32){
                  aux=aux+recortes[j][i];
              }
      }
      res.push_back(aux);
  }

return res;
}

vector<string> getvars(vector<string> pairs){

  vector<string>res;
  for (int i=0; i<pairs.size();i++){
    res.push_back(pairs[i].substr(0,pairs[i].find(":")));
  }

   for (int i=0; i<pairs.size();i++){
       res[i]="?"+res[i];
   }

  return res;
}

vector<string> getsubs(vector<string> pairs){

  vector<string>res;
  for (int i=0; i<pairs.size();i++){
     res.push_back(pairs[i].substr(pairs[i].find(":")+1,pairs[i].size()-1));
  }
  return res;
}




void compareVariables(aerostack_msgs::QueryBelief::Response res, std::map<std::string, std::string> correct_values) {
  auto sub = res.substitutions;
  vector<string> pairs= getpairs(sub);
  vector<string> vars= getvars(pairs);
  vector<string> subs= getsubs(pairs);
  ASSERT_EQ(subs.size(), correct_values.size()) <<"The expected size is not correct"; 
  for(int i=0; i<vars.size(); i++) {

    
    ASSERT_EQ(correct_values[vars[i]], subs[i]);
  }

}

/*Test 1: This test checks a simple consult*/
TEST(BeliefMenagerTest, normalCase) {
  no_beliefs();
  std::string beliefs = "object(1,plane1), color(1,red)";
  addBeliefs(beliefs,false);

  auto response = query_belief("object(?x,?),color(?x,red)");
  EXPECT_TRUE(response.success);
  compareVariables(response, {{"?x", "1"}});
  removeBeliefs(beliefs);
  no_beliefs();
}

/*Test 2: This test checks normal and multievaluated beliefs*/
TEST(BeliefMenagerTest, normalAndMultievaluatedCase) {
  no_beliefs();
  std::string beliefs = "object(1,plane101), temperature(1, 50), temperature(1, 30), temperature(1, 45)";
  addBeliefs(beliefs,false);
  auto response = query_belief("temperature(1,?x)");
  EXPECT_TRUE(response.success);
  compareVariables(response, {{"?x","45"}});

  std::string beliefs2="temperature(1, 35)";
  addBeliefs(beliefs2,true);
  auto response2 = query_belief("temperature(1,?x)");
  EXPECT_TRUE(response2.success);
  compareVariables(response2, {{"?x","45"}});

  std::string beliefs3="temperature(1, 45)";
  removeBeliefs(beliefs3);  
  auto response3 = query_belief("temperature(1,?x)");
  EXPECT_TRUE(response3.success);
  compareVariables(response3, {{"?x","35"}});
  removeBeliefs(beliefs2);

  removeBeliefs(beliefs);
  no_beliefs();
  auto response4 = query_belief("temperature(1,?x)");
  EXPECT_FALSE(response4.success);
  compareVariables(response4, {});

  no_beliefs();

}

/*Test 3: This test checks when we try to add an empty belief */
TEST(BeliefMenagerTest, caseEmpty) {
  no_beliefs();
  std::string beliefs = "";
  addBeliefs(beliefs,false);
  auto response = query_belief("color(?, ?x)");
  EXPECT_FALSE(response.success);
  compareVariables(response, {});
  removeBeliefs(beliefs);
  no_beliefs();  
}

/*Test 4: This test checks a consult that is not inside the memory of beliefs*/
TEST(BeliefMenagerTest, failedCaseWrongBelief) {
  no_beliefs();
  std::string beliefs = "nothing";
  addBeliefs(beliefs,false);
  auto response = query_belief("another(?x)");
  EXPECT_FALSE(response.success);
  compareVariables(response, {});
  removeBeliefs(beliefs);
  no_beliefs();  
}

/*Test 5: This test checks you can't consult without asking for a variable 
neither the same number of arguments*/
TEST(BeliefMenagerTest, failedCaseWrongConsult) {
  no_beliefs();
  std::string beliefs = "color(2,red)";
  addBeliefs(beliefs,false);
  auto response = query_belief("color");//
  EXPECT_FALSE(response.success);
  compareVariables(response, {});
  removeBeliefs(beliefs);
  no_beliefs();  
}

/*Test 6: This test checks beliefs are replaced in the memory of beliefs 
when they have the same predicate and the same key in a normal case*/
TEST(BeliefMenagerTest, normalCaseReplacing) {
  no_beliefs();
  std::string beliefs = "object(1,plane1), color(1,red), color(1,green)";
  addBeliefs(beliefs,false);

  auto response = query_belief("color(1,?x)");
  EXPECT_TRUE(response.success);
  compareVariables(response, {{"?x","green"}});
  removeBeliefs(beliefs);
  no_beliefs();
}

/*Test 7: This test checks if a belief is removed correctly*/
TEST(BeliefMenagerTest, normalCaseRemoving) {
  no_beliefs();
  std::string beliefs = "object(1,plane1), color(1,red), color(1,green), color(1,blue)";
  addBeliefs(beliefs,false);
  std::string removes = "color(1,blue)";
  removeBeliefs(removes);
  auto response = query_belief("color(1,?x)");
  EXPECT_FALSE(response.success);
  compareVariables(response, {});
  removeBeliefs(beliefs);
  no_beliefs();
}

/*Test 8: This test checks  multievaluated beliefs*/
TEST(BeliefMenagerTest, caseMultievalued) {
  no_beliefs();
  std::string beliefs = "object(1,plane), object(2, train), color(1,red), color(1,green), color(2,blue), color(2,red)";
  addBeliefs(beliefs,true);
  auto response = query_belief("color(1, ?x), color(2, ?x)");
  EXPECT_TRUE(response.success);
  compareVariables(response, {{"?x", "red"}});
  removeBeliefs(beliefs);
  no_beliefs();
}
/*Test 9: This test checks you can simulate a normal case with multievalued beliefs*/
TEST(BeliefMenagerTest, caseFalseMultievalued) {
  no_beliefs();
  std::string beliefs = "object(1,plane), object(2, train), color(1,red), color(1,yellow), color(2,blue), color(2,red)";
  addBeliefs(beliefs,true);
  std::string removes = "color(1,red)";
  removeBeliefs(removes);
  auto response = query_belief("color(1, ?x), color(2, ?x)");
  EXPECT_FALSE(response.success);
  compareVariables(response, {});
  removeBeliefs(beliefs);
  no_beliefs();
}
/*Test 10: This test checks a real case*/
TEST(BeliefMenagerTest, realCase) {
  no_beliefs();
  std::string beliefs = "object(1,message), sender(1,dron1),text(1,takeoff), object(2,message),sender(2,dron2), text(2,land), object(3,message), sender(3,dron1), text(3,jump)";
  addBeliefs(beliefs,true);
  auto response1 = query_belief("object(?y,message),sender(?y,dron1),text(?y,?x)");
  EXPECT_TRUE(response1.success);
  compareVariables(response1, {{"?x","takeoff"},{"?y","1"}});
  removeBeliefs("object(2,message), sender(2,dron2), text(2,land)");


  auto response2 = query_belief("object(?y,message),sender(?y,dron1),text(?y,?x)");
  EXPECT_TRUE(response2.success);
  compareVariables(response2, {{"?x","takeoff"},{"?y","1"}});

  removeBeliefs("object(1,message), sender(1,dron1), text(1,takeoff)");

  auto response3 = query_belief("object(?y,message),sender(?y,dron1),text(?y,?x)");
  EXPECT_TRUE(response3.success);
  compareVariables(response3, {{"?x","jump"},{"?y","3"}});
  removeBeliefs(beliefs);
  no_beliefs();
}

/*Test 11: This test checks that it's necessary the same number of arguments the query and the belief*/
TEST(BeliefMenagerTest, failedCaseWrongArguments) {
  no_beliefs();
  std::string beliefs = "color(1,blue)";
  addBeliefs(beliefs,false);
  auto response = query_belief("color(?x)");
  EXPECT_FALSE(response.success);
  compareVariables(response, {});
  removeBeliefs(beliefs);
  no_beliefs();  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());
  n = new ros::NodeHandle;

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  delete n;
  return result;
}
