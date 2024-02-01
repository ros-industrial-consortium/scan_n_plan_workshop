#pragma once

#include <behaviortree_cpp/tree_node.h>

namespace snp_application
{
/** @brief BT blackboard key for recording error messages from BT nodes */
inline static const std::string ERROR_MESSAGE_KEY = "error_message";

template <typename T>
T getBTInput(const BT::TreeNode* node, const std::string& port)
{
  BT::Expected<T> input = node->getInput<T>(port);
  if (!input)
    throw BT::RuntimeError("Failed to get required input value: '" + input.error() + "'");

  return input.value();
}

}  // namespace snp_application
