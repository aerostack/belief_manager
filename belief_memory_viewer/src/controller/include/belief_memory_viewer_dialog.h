/*!*******************************************************************************************
 *  \file       execution_viewer_dialog.h
 *  \brief      ExecutionViewerDialog definition file.
 *  \details    The ExecutionViewerDialog lets the user select different characteristics when
                creating a behavior or belief through the ExecutionViewer module.
 *  \author     Jorge Luis Pascual.
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

#include <ros/ros.h>
#include <droneMsgsROS/ConsultAvailableBehaviors.h>
#include <droneMsgsROS/ListOfBehaviors.h>
#include <aerostack_msgs/RequestBehaviorActivation.h>
#include <aerostack_msgs/RequestBehaviorDeactivation.h>
#include <aerostack_msgs/CheckBehaviorFormat.h>
#include <aerostack_msgs/AddBelief.h>
#include <aerostack_msgs/RemoveBelief.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>
#include <aerostack_msgs/CheckBeliefFormat.h>
#include <QString>
#include <QStringList>
#include <QList>
#include <QGridLayout>
#include <QWidget>
#include <QTableWidgetItem>
#include <QTableWidget>
#include <QDialog>
#include <QMessageBox>
#include <QString>
#include <QPoint>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/mark.h"
#include "ui_belief_memory_viewer_dialog.h"

namespace Ui
{
class BeliefMemoryViewerDialog;
}

class BeliefMemoryViewerDialog : public QDialog
{
  Q_OBJECT

public:
  explicit BeliefMemoryViewerDialog(QWidget* parent = 0, int caso = 0, QTableWidgetItem* clicked_item = 0);
  ~BeliefMemoryViewerDialog();

private:
  Ui::BeliefMemoryViewerDialog* ui;

  ros::NodeHandle n;
  ros::ServiceClient consult_available_behaviors_srv;
  ros::ServiceClient check_behavior_format_srv;
  ros::ServiceClient check_belief_format_srv;
  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient cancel_behavior_srv;
  ros::ServiceClient add_belief_srv;
  ros::ServiceClient remove_belief_srv;

  QGridLayout* layout;
  QWidget* parent;
  QTableWidgetItem* clicked_item;

  std::vector<std::string> available_behaviors;
  std::string drone_id_namespace;
  std::string consult_available_behaviors;
  std::string activate_behavior;
  std::string cancel_behavior;
  std::string check_behavior_format;
  std::string check_belief_format;
  std::string add_belief;
  std::string remove_belief;

  int caso;
  bool checkBehaviorArguments();
  bool checkBeliefFormat();

  void hideAllWidgets();
  void setUpBehaviorCombobox();

public Q_SLOTS:
  void action_accept();
  void action_cancel();

Q_SIGNALS:
};
