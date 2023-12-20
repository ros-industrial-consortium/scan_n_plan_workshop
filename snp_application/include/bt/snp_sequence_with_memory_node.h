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

#pragma once

#include "behaviortree_cpp/control_node.h"

namespace snp_application
{
/**
 * @brief Sequence control node that maintains the index of the current child node for direct re-entry in the case of halts or failures
 * @details In the case that a child node fails, the sequence re-enters at the node prior to the one that failed.
 * In the case that the sequence is halted, the sequence re-enters at the node where the sequence was halted
 */
class SNPSequenceWithMemory : public BT::ControlNode
{
public:
  inline static BT::PortsList providedPorts() { return {}; }
  SNPSequenceWithMemory(const std::string& name, const BT::NodeConfig& config);

  virtual ~SNPSequenceWithMemory() override = default;

  /**
   * Use the default halt behavior, which does not decrement `current_child_idx` by 1
   */
  using BT::ControlNode::halt;

private:
  size_t current_child_idx_;
  bool all_skipped_ = true;

  virtual BT::NodeStatus tick() override;
};

}   // namespace snp_application
