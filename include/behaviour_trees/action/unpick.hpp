/*!*******************************************************************************************
 *  \file       unpick.hpp
 *  \brief      Behaviour tree node to unpick an object
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

#ifndef UNPICK_HPP
#define UNPICK_HPP

#include <string>   

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "as2_core/names/topics.hpp"

namespace as2_behaviour_tree
{
    class Unpick : public BT::SyncActionNode
    {
    public:
        Unpick(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : BT::SyncActionNode(xml_tag_name, conf)
        {
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            callback_group_ = node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive,
                false);
            callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

            getInput("target_pose_topic_name", target_pose_topic_name_);
            getInput("info_topic_name", info_topic_name_);

            rclcpp::SubscriptionOptions sub_option;
            sub_option.callback_group = callback_group_;
            target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                target_pose_topic_name_, rclcpp::QoS(10),
                std::bind(&Unpick::targetCallback, this, std::placeholders::_1),
                sub_option);

            info_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                info_topic_name_, rclcpp::QoS(10),
                std::bind(&Unpick::infoCallback, this, std::placeholders::_1),
                sub_option);

        }

        Unpick() = delete;

        BT::NodeStatus tick()
        {
            callback_group_executor_.spin_some();
            // path_pub_->publish();
            return BT::NodeStatus::SUCCESS;

        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("target_pose_topic_name"),
                BT::InputPort<std::string>("info_topic_name")
            };
        }

    private:
        void targetCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->target_pose_ = *(msg);
        }

        void infoCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            ;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr info_sub_;

        std::string target_pose_topic_name_;
        std::string info_topic_name_;

        geometry_msgs::msg::PoseStamped target_pose_;
    };

} // namespace as2_behaviour_tree

#endif // UNPICK_HPP