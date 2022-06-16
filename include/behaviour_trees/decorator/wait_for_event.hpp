#ifndef WAIT_FOR_EVENT_CONDITION_HPP
#define WAIT_FOR_EVENT_CONDITION_HPP

#include <string>   

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

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
            sub_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
                topic_name_,
                rclcpp::SystemDefaultsQoS(),
                std::bind(&WaitForEvent::callback, this, std::placeholders::_1),
                sub_option);
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("topic_name"), BT::OutputPort<std::vector<uint8_t>>("result")};
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
        void callback(std_msgs::msg::UInt8MultiArray::SharedPtr msg)
        {
            setOutput("result", msg->data);
            flag_ = true;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
        std::string topic_name_;
        bool flag_ = false;
    };

} // namespace as2_behaviour_tree

#endif // WAIT_FOR_EVENT_CONDITION_HPP