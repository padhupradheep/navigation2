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

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "../../test_behavior_tree_fixture.hpp"
#include "../../test_dummy_tree_node.hpp"
#include "nav2_behavior_tree/plugins/decorator/is_path_longer_on_approach.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class IsPathLongerOnApproachTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("is_new_path_shorter_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::IsPathLongerOnApproach>(
          name, config);
      };

    factory_->registerBuilder<nav2_behavior_tree::IsPathLongerOnApproach>(
      "IsPathLongerOnApproach", builder);
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr IsPathLongerOnApproachTestFixture::node_ = nullptr;

BT::NodeConfiguration * IsPathLongerOnApproachTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> IsPathLongerOnApproachTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> IsPathLongerOnApproachTestFixture::tree_ = nullptr;

TEST_F(IsPathLongerOnApproachTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <IsPathLongerOnApproach path="{path}" prox_leng="16.0" length_factor="1.0">
            <AlwaysSuccess/>
          </IsPathLongerOnApproach>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // set new path on blackboard
  nav_msgs::msg::Path new_path;
  new_path.poses.resize(10);
  for (unsigned int i = 0; i < new_path.poses.size(); i++) {
    // Assuming distance between waypoints to be 1.5m
    new_path.poses[i].pose.position.x = 1.5 * i;
  }
  config_->blackboard->set<nav_msgs::msg::Path>("path", new_path);

  tree_->rootNode()->executeTick();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
