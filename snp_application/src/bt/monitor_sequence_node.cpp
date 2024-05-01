/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "snp_application/bt/monitor_sequence_node.h"

namespace snp_application
{
MonitorSequenceNode::MonitorSequenceNode(const std::string& name) :
  ControlNode::ControlNode(name, {}),
  current_child_idx_(0),
  all_skipped_(true)
{
    setRegistrationID("MonitorSequence");
}

void MonitorSequenceNode::halt()
{
  current_child_idx_ = 0;
  ControlNode::halt();
}

BT::NodeStatus MonitorSequenceNode::tick()
{
  const size_t children_count = children_nodes_.size();

  if(status() == BT::NodeStatus::IDLE)
  {
    all_skipped_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count)
  {
    TreeNode* current_child_node = children_nodes_[current_child_idx_];
    TreeNode* monitor_child_node = children_nodes_[0];

    // Always tick the button monitoring node (the first child) before ticking proceeding nodes
    if (current_child_idx_ > 0)
    {
      BT::NodeStatus monitor_status = monitor_child_node->executeTick();

      if (monitor_status == BT::NodeStatus::FAILURE)
      {
        // Reset on failure
        resetChildren();
        current_child_idx_ = 0;
        return monitor_status;
      }
    }

    auto prev_status = current_child_node->status();
    const BT::NodeStatus child_status = current_child_node->executeTick();

    // switch to RUNNING state as soon as you find an active child
    all_skipped_ &= (child_status == BT::NodeStatus::SKIPPED);

    switch (child_status)
    {
      case BT::NodeStatus::RUNNING: {
        return BT::NodeStatus::RUNNING;
      }
      case BT::NodeStatus::FAILURE: {
        // Reset on failure
        resetChildren();
        current_child_idx_ = 0;
        return child_status;
      }
      case BT::NodeStatus::SUCCESS: {
        current_child_idx_++;
        // Return the execution flow if the child is async,
        // to make this interruptable.
        if (requiresWakeUp() &&
            prev_status == BT::NodeStatus::IDLE &&
            current_child_idx_ < children_count)
        {
          emitWakeUpSignal();
          return BT::NodeStatus::RUNNING;
        }
      }
      break;

      case BT::NodeStatus::SKIPPED: {
        // It was requested to skip this node
        current_child_idx_++;
      }
      break;

      case BT::NodeStatus::IDLE: {
        throw BT::LogicError("[", name(), "]: A children should not return IDLE");
      }
    }   // end switch
  }     // end while loop

  // The entire while loop completed. This means that all the children returned SUCCESS.
  if (current_child_idx_ == children_count)
  {
    resetChildren();
    current_child_idx_ = 0;
  }
  // Skip if ALL the nodes have been skipped
  return all_skipped_ ? BT::NodeStatus::SKIPPED : BT::NodeStatus::SUCCESS;
}

}   // namespace snp_application
