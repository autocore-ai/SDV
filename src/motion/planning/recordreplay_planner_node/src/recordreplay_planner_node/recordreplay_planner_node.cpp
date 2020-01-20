// Copyright 2020 Sandro Merkli, inspired by Christopher Ho's mpc code
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
#include "recordreplay_planner_node/recordreplay_planner_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace planning
{
namespace recordreplay_planner_node
{
RecordReplayPlannerNode::RecordReplayPlannerNode(const std::string & name, const std::string & ns)
: Node{name, ns}
{
  // TODO(s.me) get topics from parameters
  const std::string tf_topic = "some_invalid_tf";
  const std::string ego_topic = "some_invalid_ego";
  const std::string trajectory_topic = "some_invalid_trajectory";
  init(ego_topic, tf_topic, trajectory_topic);
}
////////////////////////////////////////////////////////////////////////////////
RecordReplayPlannerNode::RecordReplayPlannerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & ego_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic)
: Node{name, ns}
{
  init(ego_topic, tf_topic, trajectory_topic);
}

void RecordReplayPlannerNode::init(
  const std::string & ego_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic)
{
  using rclcpp::QoS;

  // Set up subscribers
  // TODO(s.me) implement doing something with tf - probably using motion_common
  // or planning_common functionality
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_tf_sub = create_subscription<TFMessage>(tf_topic, QoS{10}.transient_local(),
      [this](const TFMessage::SharedPtr msg) {(void)msg;}, SubAllocT{});

  m_ego_sub = create_subscription<State>(ego_topic, QoS{10}.transient_local(),
      [this](const State::SharedPtr msg) {on_ego(msg);}, SubAllocT{});

  // Set up publishers
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  m_trajectory_pub =
    create_publisher<Trajectory>(trajectory_topic, QoS{10}.transient_local(), PubAllocT{});

  // Create and set a planner object that we'll talk to
  set_planner(std::make_unique<recordreplay_planner::RecordReplayPlanner>());
}


void RecordReplayPlannerNode::on_ego(const State::SharedPtr & msg)
{
  // TODO(s.me)
  // In recording mode, this would record the ego state in the planner.
  // In replay mode, this would potentially output a new trajectory in a receding horizon fashion
  (void)msg;
}


void RecordReplayPlannerNode::set_planner(PlannerPtr && planner) noexcept
{
  m_planner = std::forward<PlannerPtr &&>(planner);
}
}  // namespace recordreplay_planner_node
}  // namespace planning
}  // namespace motion