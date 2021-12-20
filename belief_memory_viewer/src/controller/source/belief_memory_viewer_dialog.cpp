/*!*******************************************************************************************
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
#include "../include/belief_memory_viewer_dialog.h"

BeliefMemoryViewerDialog::BeliefMemoryViewerDialog(QWidget* parent, int caso, QTableWidgetItem* clicked_item)
  : QDialog(parent), ui(new Ui::BeliefMemoryViewerDialog)
{
  ui->setupUi(this);

  this->layout = ui->my_layout;
  this->parent = parent;
  this->caso = caso;
  this->clicked_item = clicked_item;
  this->hideAllWidgets();

  // Nodes
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  n.param<std::string>("check_behavior_format", check_behavior_format, "check_behavior_format");
  n.param<std::string>("check_belief_format", check_belief_format, "check_belief_format");
  n.param<std::string>("consult_available_behaviors", consult_available_behaviors, "consult_available_behaviors");
  n.param<std::string>("activate_behavior", activate_behavior, "request_behavior_activation");
  n.param<std::string>("cancel_behavior", cancel_behavior, "request_behavior_deactivation");
  n.param<std::string>("add_belief", add_belief, "add_belief");
  n.param<std::string>("remove_belief", remove_belief, "remove_belief");

  // Service communications
  activate_behavior_srv =
      n.serviceClient<aerostack_msgs::RequestBehaviorActivation>('/' + drone_id_namespace + '/' + activate_behavior);
  consult_available_behaviors_srv = n.serviceClient<droneMsgsROS::ConsultAvailableBehaviors>(
      '/' + drone_id_namespace + '/' + consult_available_behaviors);
  check_belief_format_srv =
      n.serviceClient<aerostack_msgs::CheckBeliefFormat>('/' + drone_id_namespace + '/' + check_belief_format);
  add_belief_srv = n.serviceClient<aerostack_msgs::AddBelief>('/' + drone_id_namespace + '/' + add_belief);
  remove_belief_srv = n.serviceClient<aerostack_msgs::RemoveBelief>('/' + drone_id_namespace + '/' + remove_belief);
  check_behavior_format_srv =
      n.serviceClient<aerostack_msgs::CheckBehaviorFormat>('/' + drone_id_namespace + '/' + check_behavior_format);
  cancel_behavior_srv = n.serviceClient<aerostack_msgs::RequestBehaviorDeactivation>('/' + drone_id_namespace + '/' + cancel_behavior);

  this->setUpBehaviorCombobox();

  // Connects
  connect(ui->button_accept, SIGNAL(clicked()), this, SLOT(action_accept()));
  connect(ui->button_cancel, SIGNAL(clicked()), this, SLOT(action_cancel()));

  switch (this->caso)
  {
    case 0:
      // Add behavior
      {
        this->setWindowTitle(QString::fromStdString("Add a behavior"));
        ui->behavior_combobox->show();
        ui->behavior_content->show();
        this->resize(minimumSize());
        break;
      }
    case 1:
      // Add belief
      {
        this->setWindowTitle(QString::fromStdString("Add a belief"));
        ui->belief_label->show();
        ui->belief_content->show();
        ui->multivalued_label->show();
        ui->true_checkbox->show();
        ui->false_checkbox->show();
        this->resize(minimumSize());
        break;
      }
    case 2:
      // Stop behavior
      {
        this->action_accept();
        break;
      }
    case 3:
      // Remove belief
      {
        this->action_accept();
        break;
      }
  }
}

BeliefMemoryViewerDialog::~BeliefMemoryViewerDialog()
{
}

void BeliefMemoryViewerDialog::hideAllWidgets()
{
  ui->behavior_combobox->hide();
  ui->behavior_content->hide();
  ui->belief_content->hide();
  ui->belief_label->hide();
  ui->multivalued_label->hide();
  ui->true_checkbox->hide();
  ui->false_checkbox->hide();
}

void BeliefMemoryViewerDialog::setUpBehaviorCombobox()
{
  droneMsgsROS::ConsultAvailableBehaviors behaviors_message;
  droneMsgsROS::ListOfBehaviors behaviors_list;

  consult_available_behaviors_srv.call(behaviors_message);

  behaviors_list = behaviors_message.response.available_behaviors;
  available_behaviors = behaviors_list.behaviors;

  QList<QString>* available_behaviors_list = new QList<QString>();
  for (int i = 0; i < available_behaviors.size(); i++)
  {
    std::string behavior_aux = available_behaviors[i];
    available_behaviors_list->append(QString::fromStdString(behavior_aux));
  }

  QStringList definitive_list = *available_behaviors_list;
  ui->behavior_combobox->addItems(definitive_list);
}

void BeliefMemoryViewerDialog::action_accept()
{
  QMessageBox error_message;
  bool correct = false;
  switch (this->caso)
  {
    case 0:
    {
      // Add behavior
      if (!checkBehaviorArguments())
        break;

      aerostack_msgs::RequestBehaviorActivation::Request req_msg;
      aerostack_msgs::RequestBehaviorActivation::Response res_msg;
      aerostack_msgs::BehaviorCommandPriority behavior_msg;
      behavior_msg.priority = 3;
      behavior_msg.name = ui->behavior_combobox->currentText().toUtf8().constData();
      behavior_msg.arguments = ui->behavior_content->toPlainText().toUtf8().constData();
      req_msg.behavior = behavior_msg;

      bool result = activate_behavior_srv.call(req_msg, res_msg);
      correct = true;
      break;
    }
    case 1:
    {
      // Add belief
      if (!checkBeliefFormat())
        break;

      aerostack_msgs::AddBelief belief_msg;
      std::string content = ui->belief_content->toPlainText().toUtf8().constData();
      belief_msg.request.belief_expression = content;

      if (ui->true_checkbox->isChecked())
      {
        belief_msg.request.multivalued = true;
        correct = true;
      }
      else if (ui->false_checkbox->isChecked())
      {
        belief_msg.request.multivalued = false;
        correct = true;
      }
      else
      {
        error_message.setWindowTitle(QString::fromStdString("Invalid parameters"));
        error_message.setText(QString::fromStdString("Please select a multivalued option."));
        error_message.exec();
        break;
      }
      add_belief_srv.call(belief_msg);
      break;
    }
    case 2:
    {
      // Stop behavior
      aerostack_msgs::RequestBehaviorDeactivation msg;
      msg.request.name = clicked_item->text().toUtf8().constData();
      cancel_behavior_srv.call(msg);

    

      correct = true;
      break;
    }
    case 3:
    {
      // Remove belief
      QTableWidgetItem* item_to_remove = this->clicked_item;
      aerostack_msgs::RemoveBelief belief_msg;
      ;
      belief_msg.request.belief_expression = item_to_remove->text().toUtf8().constData();
      remove_belief_srv.call(belief_msg);

      correct = true;
      break;
    }
  }
  if (correct)
    this->close();
}

void BeliefMemoryViewerDialog::action_cancel()
{
  this->close();
}

bool BeliefMemoryViewerDialog::checkBehaviorArguments()
{
  QMessageBox error_message;
  aerostack_msgs::CheckBehaviorFormat msg;
  aerostack_msgs::BehaviorCommandPriority behavior_msg;

  behavior_msg.name = ui->behavior_combobox->currentText().toUtf8().constData();
  std::string arguments = ui->behavior_content->toPlainText().toUtf8().constData();
  behavior_msg.arguments = arguments;
  msg.request.behavior = behavior_msg;

  check_behavior_format_srv.call(msg);

  if (!msg.response.ack)
  {
    error_message.setWindowTitle(QString::fromStdString("Invalid behavior arguments"));
    error_message.setText(QString::fromStdString(msg.response.error_message));
    error_message.exec();
    return false;
  }
  return true;
}

bool BeliefMemoryViewerDialog::checkBeliefFormat()
{
  QMessageBox error_message;
  aerostack_msgs::CheckBeliefFormat msg;

  std::string expression = ui->belief_content->toPlainText().toUtf8().constData();
  msg.request.belief_expression = expression;

  check_belief_format_srv.call(msg);

  bool res = msg.response.success;
  if (!res)
  {
    error_message.setWindowTitle(QString::fromStdString("Invalid belief format"));
    error_message.setText(QString::fromStdString("The belief's format is invalid."));
    error_message.exec();
    return false;
  }
  return true;
}
