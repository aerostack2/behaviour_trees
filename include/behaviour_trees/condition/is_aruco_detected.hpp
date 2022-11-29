/*!*******************************************************************************************
 *  \file       is_target_detected_condition.hpp
 *  \brief      Behaviour tree node to check if target is detected and close
 *              enough
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef IS_ARUCO_DETECTED_HPP
#define IS_ARUCO_DETECTED_HPP

#include <string>

#include "as2_core/names/topics.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace as2_behaviour_tree {
class IsArucoDetectedCondition : public BT::ConditionNode {
public:
  IsArucoDetectedCondition(const std::string &xml_tag_name,
                           const BT::NodeConfiguration &conf)
      : BT::ConditionNode(xml_tag_name, conf) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    getInput("topic_name", detection_topic_name_);
    getInput("dist_threshold", threshold_);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    current_pose_sub_ =
        node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            as2_names::topics::self_localization::pose,
            as2_names::topics::self_localization::qos,
            std::bind(&IsArucoDetectedCondition::poseCallback, this,
                      std::placeholders::_1),
            sub_option);

    detection_sub_ =
        node_->create_subscription<as2_msgs::msg::PoseStampedWithID>(
            detection_topic_name_, rclcpp::SystemDefaultsQoS(),
            std::bind(&IsArucoDetectedCondition::detectionCallback, this,
                      std::placeholders::_1),
            sub_option);
  }

  IsArucoDetectedCondition() = delete;

  BT::NodeStatus tick() {
    callback_group_executor_.spin_some();
    if (is_target_) {

      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("topic_name"),
            BT::InputPort<double>("dist_threshold"),
            BT::OutputPort<std::string>("pose")};
  }

private:
  void detectionCallback(as2_msgs::msg::PoseStampedWithID::SharedPtr msg) {
    float dist = std::sqrt(
        std::pow(abs(msg->pose.position.x) - abs(this->current_pose_x_), 2.0) +
        std::pow(abs(msg->pose.position.y) - abs(this->current_pose_y_), 2.0) +
        std::pow(abs(msg->pose.position.z) - abs(this->current_pose_z_), 2.0));

    RCLCPP_DEBUG(this->node_->get_logger(), "%f", dist);
    if (dist < threshold_) {
      // geometry_msgs::msg::Pose rel_pose;
      // rel_pose.position.x = msg->pose.position.x - this->current_pose_x_;
      // rel_pose.position.y = msg->pose.position.y - this->current_pose_y_;
      // rel_pose.position.z = msg->pose.position.z - this->current_pose_z_;
      std::string output = std::to_string(msg->pose.position.x) + ";" +
                           std::to_string(msg->pose.position.y) + ";" +
                           std::to_string(msg->pose.position.z);
      setOutput("pose", output);
      is_target_ = true;
    }
  }

  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    this->current_pose_x_ = msg->pose.position.x;
    this->current_pose_y_ = msg->pose.position.y;
    this->current_pose_z_ = msg->pose.position.z;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_pose_sub_;
  std::string detection_topic_name_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr
      detection_sub_;
  double threshold_;
  bool is_target_ = false;

  std::atomic<float> current_pose_x_;
  std::atomic<float> current_pose_y_;
  std::atomic<float> current_pose_z_;
};

} // namespace as2_behaviour_tree

#endif // IS_FLYING_CONDITION_HPP