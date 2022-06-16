#ifndef LAND_ACTION_HPP
#define LAND_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "as2_core/names/actions.hpp"
#include <as2_msgs/action/land.hpp>

#include "nav2_behavior_tree/bt_action_node.hpp"

namespace as2_behaviour_tree
{
    class LandAction : public nav2_behavior_tree::BtActionNode<as2_msgs::action::Land>
    {
    public:
        LandAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : nav2_behavior_tree::BtActionNode<as2_msgs::action::Land>(xml_tag_name,
                                                                       as2_names::actions::behaviours::land, conf)
        {
        }

        void on_tick()
        {
            getInput("speed", goal_.land_speed);
        }

        void on_wait_for_result(std::shared_ptr<const as2_msgs::action::Land::Feedback> feedback)
        {
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({BT::InputPort<double>("speed")});
        }
    };

} // namespace as2_behaviour_tree

#endif // BASIC_BEHAVIOURS_HPP