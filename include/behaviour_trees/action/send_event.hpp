#ifndef SEND_EVENT_HPP
#define SEND_EVENT_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "nav2_behavior_tree/bt_service_node.hpp"

#include "as2_msgs/msg/mission_event.hpp"

namespace as2_behaviour_tree
{
    class SendEvent : public BT::SyncActionNode
    {
    public:
        SendEvent(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : BT::SyncActionNode(xml_tag_name, conf)
        {
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            callback_group_ = node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive,
                false);
            callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

            getInput("topic_name", topic_name_);

            rclcpp::PublisherOptions pub_options;
            pub_options.callback_group = callback_group_;
            pub_ = node_->create_publisher<as2_msgs::msg::MissionEvent>(
                topic_name_,
                rclcpp::SystemDefaultsQoS(),
                pub_options
            );
        }

        BT::NodeStatus tick() override
        {
            as2_msgs::msg::MissionEvent msg;
            msg.header.stamp = node_->get_clock()->now();
            getInput("data", msg.data);
            pub_->publish(msg);

            return BT::NodeStatus::SUCCESS;
        }


        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("topic_name"), BT::InputPort("data")};
        }
    
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Publisher<as2_msgs::msg::MissionEvent>::SharedPtr pub_;
        std::string topic_name_;
    };
} // namespace as2_behaviour_tree

#endif // SEND_EVENT_HPP