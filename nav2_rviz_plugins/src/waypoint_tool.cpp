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
  connect(_button1, SIGNAL(clicked()), this, SLOT(Load()));

  _button2 = new QPushButton(this);
  _button2->setText("Save WayPoints");
  connect(_button2, SIGNAL(clicked()), this, SLOT(Save()));

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
  connect(_check1, SIGNAL(stateChanged(int)), this, SLOT(loop_cb(int)));

  _line1->setDisabled(true);
  _line1->setFixedWidth(100);
  _line1->setAlignment(Qt::AlignLeft);
  _label2->setText("Number of loops");
  _hbox3->addWidget(_label2);
  _hbox3->addWidget(_line1);

  _button3 = new QPushButton(this);
  _button3->setText("Follow WayPoints");
  connect(_button3, SIGNAL(clicked()), this, SLOT(follow_waypoints()));
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

void WayPointTool::Save()
{
  if(waypoints.empty())
  {
    std::cout << "No accumulated Points to Save!" << std::endl;
    return;
  }
  std::cout << "Saving Waypoints!" << std::endl;
}

void WayPointTool::Load()
{
  std::cout << "Load lmao" << std::endl;
}

void WayPointTool::follow_waypoints()
{
  if(waypoints.empty())
  {
    std::cout << "No accumulated Points to Follow!" << std::endl;
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