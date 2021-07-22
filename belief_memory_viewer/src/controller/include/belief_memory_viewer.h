/*!*******************************************************************************************
 *  \file       execution_viewer.h
 *  \brief      Execution_viewer definition file.
 *  \details    This file displays the behaviors and beliefs used while the tree is executing.
 *  \author     Jorge Luis Pascual, Carlos Valencia.
 *  \copyright   Copyright 2019 Universidad Politecnica de Madrid (UPM)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#ifndef BELIEFMEMORYVIEWER_H
#define BELIEFMEMORYVIEWER_H

#include <ros/ros.h>
#include <droneMsgsROS/ListOfBehaviors.h>
#include <aerostack_msgs/RemoveBelief.h>
#include <aerostack_msgs/ListOfBeliefs.h>
#include <thread>
#include <chrono>

#include <QWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QGridLayout>
#include <QTextEdit>
#include <QMenu>
#include <QLabel>
#include <QSize>
#include <QCursor>
#include <QString>
#include <QRect>
#include <QGuiApplication>
#include <QScreen>
#include <QProcess>
#include <QtWidgets>


#include "ui_belief_memory_viewer.h"
#include "belief_memory_viewer_dialog.h"

#include "yaml-cpp/yaml.h"
#include <QSizePolicy>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "fstream"

namespace Ui
{
class BeliefMemoryViewer;
}

class BeliefMemoryViewer : public QWidget
{
  Q_OBJECT
public:
  explicit BeliefMemoryViewer(int argc, char** argv, QWidget* parent = 0);
  ~BeliefMemoryViewer();

  /*!********************************************************************************************************************
   *  \brief      This method refreshes the list of beliefs
   **********************************************************************************************************************/
  void listOfBeliefsCallback(const aerostack_msgs::ListOfBeliefs& msg);
  int position_x=405;
  int position_y=-395; 
private:
  Ui::BeliefMemoryViewer* ui;

  ros::NodeHandle n;
  ros::ServiceClient remove_belief_srv;
  ros::Subscriber list_of_beliefs_sub;

  QGridLayout* my_layout;
  QLabel* belief_label;
  QTableWidget* belief_content;
  QTableWidgetItem* belief_top_level_item;
  QPoint* point;
  QMenu* belief_context_menu;

  bool is_belief_context_menu_created;

  std::string remove_belief;
  std::vector<std::string> all_beliefs_content;
  std::string drone_id_namespace;
  std::string all_beliefs;

  ros::Publisher window_event_pub;
  ros::Subscriber window_event_sub;

  std::string window_event_topic;

  //aerostack_msgs::WindowEvent window_event_msg;


  boost::property_tree::ptree root;

  /*!********************************************************************************************************************
   *  \brief      This method sets up the necessary characteristics for the Behavior list to work properly
   **********************************************************************************************************************/
  void setUpBehaviorListTable();

  void clearBeliefTable();

  void resizeEvent(QResizeEvent* event);

  /*!********************************************************************************************************************
   *  \brief  This method remove single quotes and square brackets ("{" and "}") on the arguments of a behavior
   **********************************************************************************************************************/
  const std::string deSerialization(const std::string input);
  /*!********************************************************************************************************************
   *  \brief      This method is the responsible for seting up connections.
   *********************************************************************************************************************/
  void setUp();

public Q_SLOTS:

  /*!********************************************************************************************************************
   *  \brief   This slot is executed when the user right clicks anywhere inside the beliefs tree's widget.
   **********************************************************************************************************************/
  void beliefCustomContextMenu(const QPoint&);

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when the user wants to add a belief
   **********************************************************************************************************************/
  void addBelief();

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when the user wants to remove a belief.
   **********************************************************************************************************************/
  void removeBelief();
  /*!********************************************************************************************************************
   *  \brief      This method notifies main window that the widget was closed
   *********************************************************************************************************************/
  void closeEvent(QCloseEvent* event);

  /*!************************************************************************
   *  \brief   Activated when a window is closed.
   ***************************************************************************/
  //void windowOpenCallback(const aerostack_msgs::WindowEvent& msg);
  /*!************************************************************************
   *  \brief  Kills the process
   ***************************************************************************/
  void killMe();

Q_SIGNALS:

  /*!********************************************************************************************************************
   *  \brief      This signal is emitted when new beliefs info is received
   **********************************************************************************************************************/
  void setBeliefText(const QString&);
};

#endif
