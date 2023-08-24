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

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <joint_state_merger_node_parameters.hpp>


namespace joint_state_merger
{
class JointStateMergerNode : public rclcpp::Node
{
public:
  JointStateMergerNode(const rclcpp::NodeOptions &);
  ~JointStateMergerNode();

private:
  using JointStateSubscriptions =
    std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>;
  using JointStateMsgs =
    std::vector<sensor_msgs::msg::JointState::ConstSharedPtr>;

  JointStateMsgs m_subscribed_joint_states;

  std::unique_ptr<joint_state_merger_node::ParamListener> m_param_listener;
  std::unique_ptr<joint_state_merger_node::Params> m_params;

  rclcpp::TimerBase::SharedPtr m_joint_state_publish_timer;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
  JointStateSubscriptions m_joint_state_subscriptions;

  void jointStatePublishTimerCallback();
  void mergeJointStateMessage(const sensor_msgs::msg::JointState &, sensor_msgs::msg::JointState &);
};
}  // namespace joint_state_merger
