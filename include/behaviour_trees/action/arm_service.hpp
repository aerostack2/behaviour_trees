#ifndef ARM_SERVICE_HPP
#define ARM_SERVICE_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "nav2_behavior_tree/bt_service_node.hpp"

#include <std_srvs/srv/set_bool.hpp>

namespace as2_behaviour_tree
{
    class ArmService : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>
    {
    public:
        ArmService(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : nav2_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>(xml_tag_name, conf)
        {
        }

        void on_tick()
        {
            this->request_->data = true;
        }

        BT::NodeStatus on_completion(std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
    };

    class DisarmService : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>
    {
    public:
        DisarmService(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : nav2_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>(xml_tag_name, conf)
        {
        }

        void on_tick()
        {
            this->request_->data = false;
        }

        BT::NodeStatus on_completion(std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
    };

} // namespace as2_behaviour_tree

#endif // ARM_SERVICE_HPP