#ifndef GOTO_ACTION_HPP
#define GOTO_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "as2_core/names/actions.hpp"

#include <as2_msgs/action/go_to_waypoint.hpp>

#include "nav2_behavior_tree/bt_action_node.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "port_specialization.hpp"

namespace as2_behaviour_tree
{
  class GoToAction : public nav2_behavior_tree::BtActionNode<as2_msgs::action::GoToWaypoint>
  {
  public:
    GoToAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
        : nav2_behavior_tree::BtActionNode<as2_msgs::action::GoToWaypoint>(xml_tag_name,
                                                                           as2_names::actions::behaviours::gotowaypoint, conf)
    {
    }

    void on_tick()
    {
      getInput("max_speed", goal_.max_speed);
      getInput("yaw_angle", goal_.yaw_angle);
      getInput("yaw_mode", goal_.yaw_mode_flag); // TODO --> runtime warning, called BT::convertFromString() for type [unsigned char]
      getInput<geometry_msgs::msg::Pose>("pose", goal_.target_pose);
    }

    void on_wait_for_result(std::shared_ptr<const as2_msgs::action::GoToWaypoint::Feedback> feedback)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({BT::InputPort<double>("max_speed"),
                                 BT::InputPort<double>("yaw_angle"),
                                 BT::InputPort<geometry_msgs::msg::Pose>("pose"),
                                 BT::InputPort<int>("yaw_mode")});
    }
  };

} // namespace as2_behaviour_tree

#endif // GOTO_ACTION_HPP