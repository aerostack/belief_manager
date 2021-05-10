#include <gtest/gtest.h>
#include <ros/ros.h>
#include "belief_memory_viewer.h"
#include <QApplication>
#include <QtTest/QTest>
QApplication* app;
BeliefMemoryViewer* w;
int ar;
TEST(belief_memory_viewer_test, test1setup)
{
  // w->BeliefMemoryViewer::setUp();
  QVERIFY(0 == 0);
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  ar = argc;
  /*system("bash $AEROSTACK_STACK/launchers/launcher_simulated_quadrotor_basic_3.0.sh");
  system(
      "xfce4-terminal  \--tab --title \"Behavior Coordinator\"  --command \"bash -c 'roslaunch
  behavior_coordinator_process behavior_coordinator_process.launch --wait \
             my_stack_directory:=${AEROSTACK_STACK};exec bash'\" &");*/
  app = new QApplication(argc, nullptr);
  w = new BeliefMemoryViewer(argc, nullptr);
  w->show();
  return RUN_ALL_TESTS();
}
