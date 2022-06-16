#ifndef TAKEOFF_ACTION_HPP
#define TAKEOFF_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "nav2_behavior_tree/bt_action_node.hpp"

#include <as2_msgs/action/take_off.hpp>
#include "as2_core/names/actions.hpp"

namespace as2_behaviour_tree
{
  class TakeoffAction : public nav2_behavior_tree::BtActionNode<as2_msgs::action::TakeOff>
  {
  public:
    TakeoffAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
        : nav2_behavior_tree::BtActionNode<as2_msgs::action::TakeOff>(xml_tag_name,
                                                                      as2_names::actions::behaviours::takeoff, conf)
    {
    }

    void on_tick()
    {
      getInput("height", goal_.takeoff_height);
      getInput("speed", goal_.takeoff_speed);
    }

    void on_wait_for_result(std::shared_ptr<const as2_msgs::action::TakeOff::Feedback> feedback)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({BT::InputPort<double>("height"), BT::InputPort<double>("speed")});
    }

  public:
    std::string action_name_;
  };

} // namespace as2_behaviour_tree

#endif // TAKEOFF_ACTION_HPP