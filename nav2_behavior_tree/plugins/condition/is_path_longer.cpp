// Copyright (c) 2021 Neobotix GmbH
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

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/is_path_longer.hpp"

namespace nav2_behavior_tree
{

IsPathLonger::IsPathLonger(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

BT::NodeStatus IsPathLonger::tick()
{
  getInput("path", new_path);
  getInput("obstacle_spotted", obstacle_spotted);

  // Check if it is not same with the current one
  if (new_path != old_path and old_path.poses.size()!=0 ) {
    // the action server on the next loop iteration

    if ( (nav2_util::geometry_utils::calculate_path_length(old_path, 0) <
      nav2_util::geometry_utils::calculate_path_length(new_path, 0)) and 
      (old_path.poses.back() == new_path.poses.back()) ) {
      obstacle_spotted = true;
      return BT::NodeStatus::SUCCESS;
    } else {
      obstacle_spotted = false;
    }
  }
  old_path = new_path;
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPathLonger>("IsPathLonger");
}
