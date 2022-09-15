/*!*******************************************************************************************
 *  \file       unpick.hpp
 *  \brief      Behaviour tree node to pick an object
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
#include "geometry_msgs/msg/twist.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"
#include "as2_msgs/srv/package_un_pick.hpp"
#include "as2_msgs/msg/follow_target_info.hpp"

namespace as2_behaviour_tree
{
class UnPick : public BT::ActionNodeBase
{
public:
    UnPick(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
        : BT::ActionNodeBase(xml_tag_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        // Get the required items from the blackboard
        bt_loop_duration_ =
            config().blackboard->get<std::chrono::milliseconds>("bt_loop_duration");
        server_timeout_ =
            config().blackboard->get<std::chrono::milliseconds>("server_timeout");
        getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

        service_name_ = as2_names::services::behaviour::package_unpick;

        // Now that we have node_ to use, create the service client for this BT service
        service_client_ = node_->create_client<as2_msgs::srv::PackageUnPick>(
            service_name_,
            rmw_qos_profile_services_default,
            callback_group_);

        request_ = std::make_shared<as2_msgs::srv::PackageUnPick::Request>();

        getInput("target_pose_topic_name", object_pose_topic_name_);

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        // object_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     object_pose_topic_name_, rclcpp::QoS(10),
        //     std::bind(&Pick::objectCallback, this, std::placeholders::_1),
        //     sub_option);

        info_sub_ = node_->create_subscription<as2_msgs::msg::FollowTargetInfo>(
            as2_names::topics::follow_target::info,
            as2_names::topics::follow_target::qos_info,
            std::bind(&UnPick::infoCallback, this, std::placeholders::_1),
            sub_option);
    }

    UnPick() = delete;

    ~UnPick(){};

    /**
     * @brief Any subclass of BtServiceNode that accepts parameters must provide a
     * providedPorts method and call providedBasicPorts in it.
     * @param addition Additional ports to add to BT port list
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedBasicPorts(BT::PortsList addition)
    {
        BT::PortsList basic = {
            BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::string>("target_pose_topic_name")
        });
    }

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override
    {
        callback_group_executor_.spin_some();

        switch (unpick_status_)
        {
        case 0:
            if (!request_sent_)
            {
                request_->enable = true;
                geometry_msgs::msg::Twist speed_limit;
                speed_limit.linear.x = 0.0;
                speed_limit.linear.y = 0.0;
                speed_limit.linear.z = 0.0;
                request_->speed_limit = speed_limit;
                future_result_ = service_client_->async_send_request(request_);
                sent_time_ = node_->now();
                request_sent_ = true;
            }
            return check_future();
        case 1:
            return BT::NodeStatus::RUNNING;
        case 2:
            return BT::NodeStatus::SUCCESS;
        default:
            return BT::NodeStatus::FAILURE;
        }
    }

    /**
     * @brief The other (optional) override required by a BT service.
     */
    void halt() override
    {
        request_sent_ = false;
        setStatus(BT::NodeStatus::IDLE);
    }

    /**
     * @brief Check the future and decide the status of BT
     * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE otherwise
     */
    BT::NodeStatus check_future()
    {
        auto elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
        auto remaining = server_timeout_ - elapsed;

        if (remaining > std::chrono::milliseconds(0))
        {
            auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;

            rclcpp::FutureReturnCode rc;
            rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);
            if (rc == rclcpp::FutureReturnCode::SUCCESS)
            {
                // request_sent_ = false;
                return future_result_.get()->success ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
            }

            if (rc == rclcpp::FutureReturnCode::TIMEOUT)
            {
                elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
                if (elapsed < server_timeout_)
                {
                    return BT::NodeStatus::RUNNING;
                }
            }
        }

        RCLCPP_WARN(
            node_->get_logger(),
            "Node timed out while executing service call to %s.", service_name_.c_str());
        request_sent_ = false;
        return BT::NodeStatus::FAILURE;
    }

private:
    void objectCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        this->object_pose_ = *(msg);
    }

    void infoCallback(as2_msgs::msg::FollowTargetInfo::SharedPtr msg)
    {
        unpick_status_ = (int)msg->follow_status;
        unpick_mode_ = (int)msg->follow_mode;
    }

private:
    // The node that will be used for any ROS operations
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    // The timeout value while to use in the tick loop while waiting for
    // a result from the server
    std::chrono::milliseconds server_timeout_;

    // The timeout value for BT loop execution
    std::chrono::milliseconds bt_loop_duration_;

    // Pick client service node and request
    std::string service_name_;
    std::shared_ptr<rclcpp::Client<as2_msgs::srv::PackageUnPick>> service_client_;
    std::shared_ptr<as2_msgs::srv::PackageUnPick::Request> request_;

    // To track the server response when a new request is sent
    std::shared_future<as2_msgs::srv::PackageUnPick::Response::SharedPtr> future_result_;
    bool request_sent_{false};
    rclcpp::Time sent_time_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
    rclcpp::Subscription<as2_msgs::msg::FollowTargetInfo>::SharedPtr info_sub_;

    std::string object_pose_topic_name_;
    geometry_msgs::msg::PoseStamped object_pose_;

    int unpick_status_ = 0;
    int unpick_mode_ = 0;
};

} // namespace mbzirc_bt

#endif // UNPICK_HPP