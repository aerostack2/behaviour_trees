#ifndef ECHO_HPP
#define ECHO_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "nav2_behavior_tree/bt_service_node.hpp"

#include "as2_msgs/msg/mission_event.hpp"

namespace as2_behaviour_tree
{
    class Echo : public BT::SyncActionNode
    {
    public:
        Echo(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : BT::SyncActionNode(xml_tag_name, conf)
        {
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        }

        BT::NodeStatus tick() override
        {
            std::string data;
            getInput("data", data);

            RCLCPP_INFO(node_->get_logger(), "Echo: %s", data.c_str());

            return BT::NodeStatus::SUCCESS;
        }


        static BT::PortsList providedPorts()
        {
            return {BT::InputPort("data")};
        }
    
    private:
        rclcpp::Node::SharedPtr node_;
    };
} // namespace as2_behaviour_tree

#endif // ECHO_HPP