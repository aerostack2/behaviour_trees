#ifndef BASIC_BEHAVIOURS_HPP
#define BASIC_BEHAVIOURS_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"

#include <as2_msgs/action/take_off.hpp>
#include <as2_msgs/action/go_to_waypoint.hpp>
#include <as2_msgs/action/land.hpp>

#include "nav2_behavior_tree/bt_action_node.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "port_specialization.hpp"


namespace as2_behaviour_tree
{
  class TakeoffAction : public nav2_behavior_tree::BtActionNode<as2_msgs::action::TakeOff>
  {
  public:
    TakeoffAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf, std::string action_name)
        : nav2_behavior_tree::BtActionNode<as2_msgs::action::TakeOff>(xml_tag_name, action_name, conf), 
          action_name_(action_name)
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

  class GoToAction : public nav2_behavior_tree::BtActionNode<as2_msgs::action::GoToWaypoint>
  {
  public:
    GoToAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
        : nav2_behavior_tree::BtActionNode<as2_msgs::action::GoToWaypoint>(xml_tag_name, "/drone_sim_0/GoToWaypointBehaviour", conf)
    {
    }

    void on_tick()
    {
      getInput("max_speed", goal_.max_speed);
      getInput("yaw_angle", goal_.yaw_angle);
      getInput("yaw_mode", goal_.yaw_mode_flag);  // TODO --> runtime warning, called BT::convertFromString() for type [unsigned char]
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
                                 BT::InputPort<int>("yaw_mode")}
      );
    }
  };

  class LandAction : public nav2_behavior_tree::BtActionNode<as2_msgs::action::Land>
  {
  public:
    LandAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
        : nav2_behavior_tree::BtActionNode<as2_msgs::action::Land>(xml_tag_name, "/drone_sim_0/LandBehaviour", conf)
    {
    }

    void on_tick()
    {
      getInput("land_speed", goal_.land_speed);
    }

    void on_wait_for_result(std::shared_ptr<const as2_msgs::action::Land::Feedback> feedback)
    {
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts({BT::InputPort<double>("land_speed")});
    }
  };

} // namespace as2_behaviour_tree

#endif // BASIC_BEHAVIOURS_HPP