/*!*******************************************************************************************
 *  \file       wait_for_event.hpp
 *  \brief      Wait for event implementation as behaviour tree node
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

#ifndef WAIT_FOR_EVENT_CONDITION_HPP
#define WAIT_FOR_EVENT_CONDITION_HPP

#include <string>   

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"
#include "as2_msgs/msg/mission_event.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace as2_behaviour_tree
{
    class WaitForEvent : public BT::DecoratorNode
    {
    public:
        WaitForEvent(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : BT::DecoratorNode(xml_tag_name, conf)
        {
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            callback_group_ = node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive,
                false);
            callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

            getInput("topic_name", topic_name_);

            rclcpp::SubscriptionOptions sub_option;
            sub_option.callback_group = callback_group_;
            sub_ = node_->create_subscription<as2_msgs::msg::MissionEvent>(
                topic_name_,
                rclcpp::SystemDefaultsQoS(),
                std::bind(&WaitForEvent::callback, this, std::placeholders::_1),
                sub_option);
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("topic_name"), BT::OutputPort("result")};
        }

    private:
        BT::NodeStatus tick() override
        {
            callback_group_executor_.spin_some();
            if (flag_) {
                return child_node_->executeTick();
            }
            return BT::NodeStatus::RUNNING;
        }

    private:
        void callback(as2_msgs::msg::MissionEvent::SharedPtr msg)
        {
            setOutput("result", msg->data);
            flag_ = true;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<as2_msgs::msg::MissionEvent>::SharedPtr sub_;
        std::string topic_name_;
        bool flag_ = false;
    };

} // namespace as2_behaviour_tree

#endif // WAIT_FOR_EVENT_CONDITION_HPP