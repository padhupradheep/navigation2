// Copyright (c) 2022 Avery Girven
// Copyright (c) 2022 University of Michigan -Dearborn
// Copyright (c) 2022 Neobotix GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_rviz_plugins/waypoint_tool.hpp"
#include "nav2_rviz_plugins/goal_common.hpp"
#include "nav2_rviz_plugins/goal_pose_updater.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>
#include <memory>
#include <QDebug> 
#include <QFileDialog>
#include <fstream>
#include "yaml-cpp/yaml.h"

namespace nav2_rviz_plugins
{
using nav2_util::geometry_utils::orientationAroundZAxis;

WayPointTool::WayPointTool(QWidget * parent)
: Panel(parent)
{
  _vbox = new QVBoxLayout();
  _hbox1 = new QHBoxLayout();
  _hbox2 = new QHBoxLayout();
  _hbox3 = new QHBoxLayout();
  _hbox4 = new QHBoxLayout();
  _hbox5 = new QHBoxLayout();

  QFrame * _line = new QFrame();
  _line->setFrameShape(QFrame::HLine);
  _line->setFrameShadow(QFrame::Sunken);

  _button1 = new QPushButton(this);
  _button1->setText("Load WayPoints");
  connect(_button1, SIGNAL(clicked()), this, SLOT(load()));

  _button2 = new QPushButton(this);
  _button2->setText("Save WayPoints");
  connect(_button2, SIGNAL(clicked()), this, SLOT(save()));

  _check1 = new QCheckBox();
  _check1->setChecked(false);

  _line1 = new QLineEdit(this);
  _label1 = new QLabel(this);
  _label2 = new QLabel(this);

  _hbox1->addWidget(_button1);
  _hbox1->addWidget(_button2);

  _label1->setText("Loop WayPoints");
  _hbox2->addWidget(_label1);
  _hbox2->addWidget(_check1);
  _hbox2->setSpacing(10);
  // connect(_check1, SIGNAL(stateChanged(int)), this, SLOT(loop_cb(int)));

  _line1->setDisabled(true);
  _line1->setFixedWidth(100);
  _line1->setAlignment(Qt::AlignLeft);
  _label2->setText("Number of loops");
  _hbox3->addWidget(_label2);
  _hbox3->addWidget(_line1);

  _button3 = new QPushButton(this);
  _button3->setText("Follow WayPoints");
  // connect(_button3, SIGNAL(clicked()), this, SLOT(follow_waypoints()));
  _hbox5->addWidget(_button3);

  _vbox->addLayout(_hbox1);
  _vbox->addLayout(_hbox2);
  _vbox->addLayout(_hbox3);
  //_vbox->addLayout(_hbox4);
  _vbox->addLayout(_hbox5);

  _vbox->setContentsMargins(5, 5, 5, 5);
  setLayout(_vbox);

  QObject::connect(
    &GoalUpdater, SIGNAL(updateGoal(double,double,double,QString)), // NOLINT
    this, SLOT(accumalate_points(double,double,double,QString)));  // NOLINT
}

void WayPointTool::save()
{
  if(waypoints.empty())
  {
    qDebug() << "No accumulated Points to Save!";
    return;
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "waypoints";

  for (unsigned int i = 0; i < waypoints.size(); ++i) {
    out << YAML::BeginMap;
    out << YAML::Key << "pose";
    std::vector<double> pose = {waypoints[i].pose.position.x, waypoints[i].pose.position.y, waypoints[i].pose.position.z};
    out << YAML::Value << pose;
    out << YAML::Key << "orientation";
    std::vector<double> orientation = {waypoints[i].pose.orientation.w, waypoints[i].pose.orientation.x, waypoints[i].pose.orientation.y, waypoints[i].pose.orientation.z};
    out << YAML::Value << orientation;
  }

  std::ofstream fout("file.yaml");
  fout << out.c_str();

  // save waypoints to data structure 
  std::cout << "Saving Waypoints!" << std::endl;
}

void WayPointTool::load()
{
  QString file = QFileDialog::getOpenFileName(this,
        tr("Open File"), "",
        tr("yaml(*.yaml);;All Files (*)"));

  YAML::Node available_waypoints = YAML::LoadFile("waypoints.yaml");
  
  const YAML::Node& waypoint_iter = available_waypoints["waypoints"];
  for (YAML::const_iterator it = waypoint_iter.begin(); it != waypoint_iter.end(); ++it) {
    const YAML::Node& waypoint = *it;
    auto pose = waypoint["pose"].as<std::vector<double>>();
    auto orientation = waypoint["orientation"].as<std::vector<double>>();
    waypoints.push_back(convert_to_msg(pose, orientation));
  }
}

geometry_msgs::msg::PoseStamped WayPointTool::convert_to_msg(
  std::vector<double> pose,
  std::vector<double> orientation)
{
  auto msg = geometry_msgs::msg::PoseStamped();

  msg.header.frame_id = "map";
  msg.header.stamp = rclcpp::Clock().now();

  msg.pose.position.x = pose[0];
  msg.pose.position.y = pose[1];
  msg.pose.position.z = pose[2];
  
  msg.pose.orientation.w = orientation[0];
  msg.pose.orientation.w = orientation[1];
  msg.pose.orientation.w = orientation[2];
  msg.pose.orientation.w = orientation[3];

  return msg;
}

void WayPointTool::follow_waypoints()
{
  if(waypoints.empty())
  {
    qDebug() << "No accumulated Points to Follow!";
    return;
  }
  auto feedback = std::make_shared<nav2_msgs::action::FollowWaypoints::Feedback>();
  
  std::vector<geometry_msgs::msg::PoseStamped> poses = waypoints;
  auto is_action_server_ready =
    waypoint_follower_action_client->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    return;
  }
  waypoint_follower_goal.poses = poses;

  RCLCPP_INFO(ros_node_->get_logger(),
    "Sending a path of %zu waypoints:)",
    waypoint_follower_goal.poses.size());
  for (auto waypoint : waypoint_follower_goal.poses) {
    RCLCPP_DEBUG(ros_node_->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
         
  auto future_goal_handle =
    waypoint_follower_action_client->async_send_goal(waypoint_follower_goal, send_goal_options);  
  // Get the goal handle and save so that we can check on completion
  waypoint_follower_goal_handle = future_goal_handle.get();
  
  if (waypoint_follower_goal_handle) {
    return;
  }
}

void WayPointTool::loop_cb(int state)
{
  if(state == 2)
    _line1->setDisabled(false);
  else
    _line1->setDisabled(true); 
}

void WayPointTool::accumalate_points(double x, double y, double theta, QString frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame.toStdString();
  pose.header.stamp = rclcpp::Clock().now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  pose.pose.orientation = orientationAroundZAxis(theta);

  
  waypoints.push_back(pose);
}

}  // namespace WayPointTool

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::WayPointTool, rviz_common::Panel)