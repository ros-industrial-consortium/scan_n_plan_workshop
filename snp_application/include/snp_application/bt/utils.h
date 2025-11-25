#pragma once

#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/node.hpp>

namespace snp_application
{
/** @brief BT blackboard key for recording error messages from BT nodes */
inline static const std::string ERROR_MESSAGE_KEY = "error_message";

inline static const std::string WARN_MESSAGE_KEY = "warn_message";

template <typename T>
T getBTInput(const BT::TreeNode* node, const std::string& port)
{
  BT::Expected<T> input = node->getInput<T>(port);
  if (!input)
    throw BT::RuntimeError("Failed to get required input value: '" + input.error() + "'");

  return input.value();
}

template <typename T>
T get_parameter(rclcpp::Node::SharedPtr node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

}  // namespace snp_application
