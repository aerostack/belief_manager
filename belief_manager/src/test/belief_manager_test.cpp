#include "../include/belief_manager.h"
#include <gtest/gtest.h>

TEST(QueryTests, noBacktracking0) {
  BeliefManager manager;

  manager.addBeliefs("location(drone1, (2,5,1.5)), color(drone1, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("location(drone1, ?x), color(drone1, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?x"], "(2, 5, 1.5)");
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, noBacktracking1) {
  BeliefManager manager;

  manager.addBeliefs("location(drone1, (2,5,1.5)), color(drone1, red), color(myCar, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("location(drone1, ?x), color(drone1, ?y), color(myCar, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?x"], "(2, 5, 1.5)");
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, noBacktracking2) {
  BeliefManager manager;

  manager.addBeliefs("color(drone1, red), color(myCar, blue), color(myCar, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(drone1, ?y), color(myCar, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, noBacktracking3) {
  BeliefManager manager;

  manager.addBeliefs("location(drone1, (2,5,1.5)), color(drone1, red), color(myCar, blue)", true);

  BeliefManager::QueryResult result = manager.executeQuery("location(drone1, ?x), color(drone1, ?y), color(myCar, ?y)");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.variables.empty());
}

TEST(QueryTests, backtracking0) {
  BeliefManager manager;

  manager.addBeliefs("color(myCar, blue), color(drone1, red), color(myCar, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(myCar, ?y), color(drone1, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, backtracking1) {
  BeliefManager manager;

  manager.addBeliefs("color(myCar, blue), tint(drone1, red), color(myCar, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(myCar, ?y), tint(?x, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?x"], "drone1");
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, backtracking2) {
  BeliefManager manager;

  manager.addBeliefs("color(myCar, blue), color(myCar, green), tint(drone1, red), color(myCar, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(myCar, ?y), tint(?, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, backtracking3) {
  BeliefManager manager;

  manager.addBeliefs("color(drone1, blue), color(myCar, green), tint(drone1, red), color(drone1, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(?x, ?y), tint(?x, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, backtracking4) {
  BeliefManager manager;

  manager.addBeliefs("color(drone1, blue), color(myCar, green), tint(drone1, red), color(drone1, yellow)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(?x, ?y), tint(?x, ?y)");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.variables.empty());
}

TEST(QueryTests, backtracking5) {
  BeliefManager manager;

  manager.addBeliefs("color(myCar, blue), color(drone1, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(myCar, ?y), color(drone1, ?y)");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.variables.empty());
}

TEST(QueryTests, backtracking6) {
  BeliefManager manager;

  manager.addBeliefs("color(myCar, blue), color(drone1, red), color(myCar, red)", true);

  BeliefManager::QueryResult result = manager.executeQuery("color(myCar, ?y), color(drone1, ?y)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?y"], "red");
}

TEST(QueryTests, backtracking7) {
  BeliefManager manager;

  manager.addBeliefs("test(a), test(b), test((1,2,3), b), test((1,3,3), a)", true);

  BeliefManager::QueryResult result = manager.executeQuery("test(?x), test((?,2,?), ?x), test(?y, ?x)");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.variables["?x"], "b");
  EXPECT_EQ(result.variables["?y"], "(1, 2, 3)");
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
