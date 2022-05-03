// Copyright (c) 2022 Avery Girven
// Copyright (c) 2022 University of Michigan -Dearborn
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

#ifndef NAV2_RVIZ_PLUGINS__WAYPOINT_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__WAYPOINT_TOOL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_rviz_plugins/goal_pose_updater.hpp"

#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>
#include <QRadioButton>

class QLineEdit;
class QSpinBox;
class QComboBox;

namespace rviz_common
{
class VisualizationManager;
}  // namespace rviz_common

namespace nav2_rviz_plugins
{
class WayPointTool : public rviz_common::Panel
{
  Q_OBJECT

public:
  using WaypointFollowerGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  explicit WayPointTool(QWidget * parent = 0);

private Q_SLOTS:
  /**
   * @brief load in a prevoiously saved set of waypoints
   */
  void load();

  /**
   * @brief Save the current ordered set of waypoints
   */
  void save();

  /**
   * @brief publish the accumulated set of waypoints over the waypoint follower action server
   */
  void follow_waypoints();

  /**
   * @brief 
   * @param state 
   */
  void loop_cb(int state);

  /**
   * @brief accumalate an ordered set of waypoints from the goal tool
   * @param x 
   * @param y 
   * @param theta 
   * @param frame 
   */
  void accumalate_points(double x, double y, double theta, QString frame);

protected:

  // qt box
  QVBoxLayout * _vbox;
  QHBoxLayout * _hbox1;
  QHBoxLayout * _hbox2;
  QHBoxLayout * _hbox3;
  QHBoxLayout * _hbox4;
  QHBoxLayout * _hbox5;

  // qt buttons
  QPushButton * _button1;
  QPushButton * _button2;
  QPushButton * _button3;
  QCheckBox * _check1;

  // qt line
  QLineEdit * _line1;

  // qt labels
  QLabel * _label1;
  QLabel * _label2;

  QFrame * _line;

  // ros 
  rclcpp::Node::SharedPtr ros_node_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_action_client;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle;
  std::unique_ptr<std::thread> _thread;
  GoalPoseUpdater GoalUpdater;
};

}  // namespace WayPointTool
#endif  // NAV2_RVIZ_PLUGINS__WAYPOINT_TOOL_HPP_