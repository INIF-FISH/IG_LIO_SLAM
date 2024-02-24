/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "nav2_costmap_2d/layer.hpp"

#include <string>
#include <vector>
#include "nav2_util/node_utils.hpp"

namespace nav2_costmap_2d
{

Layer::Layer()
: layered_costmap_(nullptr),
  name_(),
  tf_(nullptr),
  current_(false),
  enabled_(false)
{}

void
Layer::initialize(
  LayeredCostmap * parent,
  std::string name,
  tf2_ros::Buffer * tf,
  const nav2_util::LifecycleNode::WeakPtr & node,
  rclcpp::Node::SharedPtr client_node,
  rclcpp::Node::SharedPtr rclcpp_node)
{
  layered_costmap_ = parent;
  name_ = name;
  tf_ = tf;
  client_node_ = client_node;
  rclcpp_node_ = rclcpp_node;
  node_ = node;

  {
    auto node_shared_ptr = node_.lock();
    logger_ = node_shared_ptr->get_logger();
    clock_ = node_shared_ptr->get_clock();
  }

  onInitialize();
}

const std::vector<geometry_msgs::msg::Point> &
Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();
}

void
Layer::declareParameter(
  const std::string & param_name,
  const rclcpp::ParameterValue & value)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);
  nav2_util::declare_parameter_if_not_declared(
    node, getFullName(param_name), value);
}

void
Layer::declareParameter(
  const std::string & param_name,
  const rclcpp::ParameterType & param_type)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);
  nav2_util::declare_parameter_if_not_declared(
    node, getFullName(param_name), param_type);
}

bool
Layer::hasParameter(const std::string & param_name)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  return node->has_parameter(getFullName(param_name));
}

std::string
Layer::getFullName(const std::string & param_name)
{
  return std::string(name_ + "." + param_name);
}

}  // end namespace nav2_costmap_2d
