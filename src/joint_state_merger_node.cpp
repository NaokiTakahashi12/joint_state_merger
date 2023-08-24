// MIT License
//
// Copyright (c) 2023 NaokiTakahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"), to deal in the Software
// without restriction, including without limitation the rights to use, copy, modify,
// merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <joint_state_merger/joint_state_merger_node.hpp>

#include <memory>
#include <chrono>
#include <functional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <joint_state_merger_node_parameters.hpp>


namespace joint_state_merger
{
JointStateMergerNode::JointStateMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("joint_state_merger", node_options),
  m_subscribed_joint_states(),
  m_param_listener(nullptr),
  m_params(nullptr),
  m_joint_state_publish_timer(nullptr),
  m_joint_state_publisher(nullptr),
  m_joint_state_subscriptions()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

  m_param_listener = std::make_unique<joint_state_merger_node::ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = std::make_unique<joint_state_merger_node::Params>(
    m_param_listener->get_params()
  );

  m_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_states",
    rclcpp::QoS(5)
  );
  if (0 >= m_params->joint_state_sources.size()) {
    RCLCPP_ERROR(this->get_logger(), "Not set joitn state sources");
    rclcpp::shutdown();
  }
  int subscription_index = 0;
  m_subscribed_joint_states.resize(m_params->joint_state_sources.size());
  for (const auto & subscribe_topic : m_params->joint_state_sources) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscribe topic " << subscribe_topic);
    m_joint_state_subscriptions.push_back(
      this->create_subscription<sensor_msgs::msg::JointState>(
        subscribe_topic,
        rclcpp::QoS(5),
        [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          m_subscribed_joint_states[subscription_index].reset();
          m_subscribed_joint_states[subscription_index] = msg;
        }
      )
    );
    subscription_index++;
  }
  const unsigned int joint_state_publish_duration_ms = 1e3 / m_params->publish_frequency;
  m_joint_state_publish_timer = this->create_wall_timer(
    std::chrono::milliseconds(
      joint_state_publish_duration_ms
    ),
    std::bind(
      &JointStateMergerNode::jointStatePublishTimerCallback,
      this
    )
  );
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize successful " << this->get_name());
}

JointStateMergerNode::~JointStateMergerNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void JointStateMergerNode::jointStatePublishTimerCallback()
{
  bool exist_merge_source = false;
  for (const auto & joint_state : m_subscribed_joint_states) {
    if (joint_state) {
      exist_merge_source = true;
      break;
    }
  }
  if (not exist_merge_source) {
    return;
  }
  auto pub_msg = std::make_unique<sensor_msgs::msg::JointState>();
  pub_msg->header.stamp = this->get_clock()->now();

  for (const auto & joint_state : m_subscribed_joint_states) {
    if (not joint_state) {
      continue;
    }
    mergeJointStateMessage(*joint_state, *pub_msg);
  }
  m_joint_state_publisher->publish(std::move(pub_msg));
}

template<typename Scalar>
void appendIfExistsVector(
  const std::vector<Scalar> & source,
  int source_index,
  std::vector<Scalar> & target)
{
  if (source.size() <= static_cast<std::size_t>(source_index)) {
    target.push_back(std::nan(""));
    return;
  }
  target.push_back(source[source_index]);
}

void JointStateMergerNode::mergeJointStateMessage(
  const sensor_msgs::msg::JointState & source,
  sensor_msgs::msg::JointState & target)
{
  int source_index = -1;
  for (const auto & name : source.name) {
    source_index++;
    const auto has_key_count = std::count(target.name.cbegin(), target.name.cend(), name);
    if (1 == has_key_count) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Skipped merge joint key " << name);
      continue;
    } else if (1 < has_key_count) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Duplication name key: " << name);
    }
    target.name.push_back(source.name[source_index]);
    appendIfExistsVector(source.position, source_index, target.position);
    appendIfExistsVector(source.velocity, source_index, target.velocity);
    appendIfExistsVector(source.effort, source_index, target.effort);
  }
}
}  // namespace joint_state_merger

RCLCPP_COMPONENTS_REGISTER_NODE(joint_state_merger::JointStateMergerNode)
