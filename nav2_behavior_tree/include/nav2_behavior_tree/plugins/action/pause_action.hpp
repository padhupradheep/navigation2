// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PAUSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PAUSE_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/pause.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::FollowPath
 */
class PauseAction : public BtActionNode<nav2_msgs::action::Pause>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::Pause
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  PauseAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   */
  // void on_wait_for_result() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  nav_msgs::msg::Path old_path;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
        BT::InputPort<bool>("obstacle_spotted", false, "obstacle check")
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PAUSE_ACTION_HPP_