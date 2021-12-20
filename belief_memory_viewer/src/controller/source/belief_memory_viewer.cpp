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
/*
  ExecutionViewer
  @author  Jorge Luis Pascual, Carlos Valencia.
  @date    07-2017
  @version 2.0
*/
#include "../include/belief_memory_viewer.h"

BeliefMemoryViewer::BeliefMemoryViewer(int argc, char** argv, QWidget* parent) : QWidget(parent), ui(new Ui::BeliefMemoryViewer)
{
  QWidget::setLocale(QLocale());

  ui->setupUi(this);

  // window always on top
  Qt::WindowFlags flags = windowFlags();
  setWindowFlags(flags | Qt::WindowStaysOnTopHint);
  setWindowIcon(QIcon(":/img/img/execution_viewer.png"));
  setWindowTitle("Belief Memory Viewer");
  qRegisterMetaType<QVector<int>>("QVector<int>");
  my_layout = ui->gridLayout;
  this->point = 0;

  belief_label = new QLabel("Belief Viewer", this);

  belief_content = new QTableWidget(this);

  QSizePolicy behavior_policy = QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  QSizePolicy belief_policy = QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  behavior_policy.setVerticalStretch(1);
  belief_policy.setVerticalStretch(5);


  belief_content->setSizePolicy(belief_policy);

  my_layout->addWidget(belief_label, 0, 0);
  my_layout->addWidget(belief_content, 1, 0);

  belief_label->show();
  belief_content->show();

  this->is_belief_context_menu_created = false;

  n.param<std::string>("remove_belief", remove_belief, "remove_belief");
  n.param<std::string>("all_beliefs", all_beliefs, "all_beliefs");
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");

  remove_belief_srv = n.serviceClient<aerostack_msgs::RemoveBelief>(remove_belief);
  list_of_beliefs_sub =
      n.subscribe("/" + drone_id_namespace + "/" + all_beliefs, 1000, &BeliefMemoryViewer::listOfBeliefsCallback, this);

  setUpBehaviorListTable();

  // reads layout file
  namespace pt = boost::property_tree;

  //std::string layout_dir = std::getenv("AEROSTACK_STACK") + std::string("/stack/ground_control_system/"
                                                                       // "graphical_user_interface/layouts/layout.json");

  //pt::read_json(layout_dir, root);

  QScreen* screen = QGuiApplication::primaryScreen();
  QRect screenGeometry = screen->geometry();

  int y0 = screenGeometry.height() / 2;
  int x0 = screenGeometry.width() / 2;
  int height = 500;//root.get<int>("EXECUTION_VIEWER.height");
  int width = 500;//root.get<int>("EXECUTION_VIEWER.width");

  this->resize(width, height);
  this->move(x0 + position_x/*root.get<int>("EXECUTION_VIEWER.position.x")*/, y0 + position_y/*root.get<int>("EXECUTION_VIEWER.position.y")*/);

  // Settings connections
  setUp();
}

BeliefMemoryViewer::~BeliefMemoryViewer()
{
  delete ui;
}

void BeliefMemoryViewer::setUp()
{
  n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  n.param<std::string>("window_event_topic", window_event_topic, "window_event");


  // Subscribers
  //window_event_sub =
    //  n.subscribe("/" + drone_id_namespace + "/" + window_event_topic, 10, &BeliefMemoryViewer::windowOpenCallback, this);

  // Publishers
  //window_event_pub =
   //   n.advertise<aerostack_msgs::WindowEvent>("/" + drone_id_namespace + "/" + window_event_topic, 1, true);
}


void BeliefMemoryViewer::listOfBeliefsCallback(const aerostack_msgs::ListOfBeliefs& msg)
{
  this->belief_content->setEditTriggers(QAbstractItemView::NoEditTriggers);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Needed to adapt all_beliefs topic frequency to GUI
                                                                // frequency
  this->clearBeliefTable();
  this->all_beliefs_content.clear();
  std::stringstream stream(msg.beliefs);
  std::string one_belief;

  int i = 0;
  while (std::getline(stream, one_belief))
  {
    // std::cout<<one_belief<<std::endl;
    this->belief_content->setRowCount(i + 1);
    all_beliefs_content.push_back(one_belief);
    QTableWidgetItem* new_belief = new QTableWidgetItem(QString::fromStdString(one_belief));
    this->belief_content->setItem(i, 0, new_belief);
    i++;
  }
}

void BeliefMemoryViewer::setUpBehaviorListTable()
{
  QStringList* headers = new QStringList();
  headers->append("Behavior Name");
  headers->append("Arguments");

  this->belief_content->setColumnCount(1);
  this->belief_content->verticalHeader()->setVisible(false);
  this->belief_content->horizontalHeader()->setVisible(false);
  this->belief_content->setSortingEnabled(false);
  this->belief_content->adjustSize();
  this->belief_content->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(belief_content, SIGNAL(customContextMenuRequested(const QPoint&)), this,
          SLOT(beliefCustomContextMenu(const QPoint&)));
  this->belief_content->viewport()->setFocusPolicy(Qt::NoFocus);
}

void BeliefMemoryViewer::beliefCustomContextMenu(const QPoint& p)
{
  if (this->belief_content->itemAt(p) != 0)
  {
    if (is_belief_context_menu_created)
    {
      belief_context_menu->close();
      delete belief_context_menu;
      is_belief_context_menu_created = false;
    }
    belief_context_menu = new QMenu(tr("Menu"), this->belief_content);
    is_belief_context_menu_created = true;
    this->point = new QPoint(p);

    QAction action_add_belief("Add a belief", this->belief_content);
    QAction action_remove_belief("Remove this belief", this->belief_content);

    belief_context_menu->addAction(&action_add_belief);
    belief_context_menu->addAction(&action_remove_belief);

    connect(&action_add_belief, SIGNAL(triggered()), this, SLOT(addBelief()));
    connect(&action_remove_belief, SIGNAL(triggered()), this, SLOT(removeBelief()));

    belief_context_menu->exec(QCursor::pos());
  }
  else
  {
    if (is_belief_context_menu_created)
    {
      belief_context_menu->close();
      delete belief_context_menu;
      is_belief_context_menu_created = false;
    }
    belief_context_menu = new QMenu(tr("Menu"), this->belief_content);
    is_belief_context_menu_created = true;
    this->point = 0;

    QAction action_add_belief("Add a belief", this->belief_content);

    belief_context_menu->addAction(&action_add_belief);

    connect(&action_add_belief, SIGNAL(triggered()), this, SLOT(addBelief()));

    belief_context_menu->exec(QCursor::pos());
  }
}

void BeliefMemoryViewer::addBelief()
{
  BeliefMemoryViewerDialog* dialog = new BeliefMemoryViewerDialog(this, 1, 0);
  dialog->show();
}


void BeliefMemoryViewer::removeBelief()
{
  BeliefMemoryViewerDialog* dialog = new BeliefMemoryViewerDialog(this, 3, belief_content->itemAt(*point));
}

void BeliefMemoryViewer::clearBeliefTable()
{
  this->belief_content->setRowCount(0);
}

void BeliefMemoryViewer::resizeEvent(QResizeEvent* event)
{
  this->belief_content->setColumnWidth(0, this->belief_content->width());
  // this->belief_content->setColumnWidth(1, this->belief_content->width()/2);
  QWidget::resizeEvent(event);
}

const std::string BeliefMemoryViewer::deSerialization(const std::string input)
{
  std::string de_serialization_result;
  for (int j = 0; j < input.size(); j++)
  {
    if (input[j] != '\'' && input[j] != '}' && input[j] != '{')
    {
      de_serialization_result = de_serialization_result + input[j];
    }
  }
  return de_serialization_result;
}
void BeliefMemoryViewer::closeEvent(QCloseEvent* event)
{
  //window_event_msg.window = aerostack_msgs::WindowEvent::EXECUTION_VIEWER;
  //window_event_msg.event = aerostack_msgs::WindowEvent::CLOSE;
  //window_event_pub.publish(window_event_msg);
}

void BeliefMemoryViewer::killMe()
{
#ifdef Q_OS_WIN
  enum
  {
    ExitCode = 0
  };
  ::TerminateProcess(::GetCurrentProcess(), ExitCode);
#else
  qint64 pid = QCoreApplication::applicationPid();
  QProcess::startDetached("kill -9 " + QString::number(pid));
#endif  // Q_OS_WIN
}

/*void BeliefMemoryViewer::windowOpenCallback(const aerostack_msgs::WindowEvent& msg)
{
 
  if ( msg.window == aerostack_msgs::WindowEvent::ALPHANUMERIC_INTERFACE_CONTROL )
  {
     window_event_msg.window = aerostack_msgs::WindowEvent::EXECUTION_VIEWER;
     window_event_msg.event = aerostack_msgs::WindowEvent::CLOSE;
     window_event_pub.publish(window_event_msg);
     killMe();
  }

  if (msg.window == aerostack_msgs::WindowEvent::INTEGRATED_VIEWER && msg.event == aerostack_msgs::WindowEvent::MINIMIZE)
    showMinimized();
}*/
