#ifndef IS_FLYING_CONDITION_HPP
#define IS_FLYING_CONDITION_HPP

#include <string>   

#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/msg/platform_status.hpp"
#include "as2_core/names/topics.hpp"

namespace as2_behaviour_tree
{
    class IsFlyingCondition : public BT::ConditionNode
    {
    public:
        IsFlyingCondition(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : BT::ConditionNode(xml_tag_name, conf)
        {
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            callback_group_ = node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive,
                false);
            callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

            rclcpp::SubscriptionOptions sub_option;
            sub_option.callback_group = callback_group_;
            state_sub_ = node_->create_subscription<as2_msgs::msg::PlatformInfo>(
                as2_names::topics::platform::info,
                as2_names::topics::platform::qos,
                std::bind(&IsFlyingCondition::stateCallback, this, std::placeholders::_1),
                sub_option);
        }

        IsFlyingCondition() = delete;

        BT::NodeStatus tick()
        {
            callback_group_executor_.spin_some();
            if (is_flying_) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
        void stateCallback(as2_msgs::msg::PlatformInfo::SharedPtr msg)
        {
            is_flying_ = msg->status.state == as2_msgs::msg::PlatformStatus::FLYING;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr state_sub_;
        bool is_flying_ = false;
    };

} // namespace as2_behaviour_tree

#endif // IS_FLYING_CONDITION_HPP