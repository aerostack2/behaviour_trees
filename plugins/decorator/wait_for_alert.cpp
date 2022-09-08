/*!*******************************************************************************************
 *  \file       wait_for_alert.cpp
 *  \brief      Wait for alert implementation as behaviour tree node
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Javier Melero Deza
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

#include "behaviour_trees/decorator/wait_for_alert.hpp"

namespace as2_behaviour_tree
{
    WaitForAlert::WaitForAlert(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
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
        sub_ = node_->create_subscription<as2_msgs::msg::Alert>(
            topic_name_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&WaitForAlert::callback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus WaitForAlert::tick()
    {
        callback_group_executor_.spin_some();
        if (flag_)
        {
            return child_node_->executeTick();
        }
        return BT::NodeStatus::RUNNING;
    }

    void WaitForAlert::callback(as2_msgs::msg::Alert::SharedPtr msg)
    {
        setOutput("alert", std::to_string(msg->alert_code));
        flag_ = true;
    }

} // namespace as2_behaviour_tree
