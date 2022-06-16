#include <chrono>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "behaviour_trees/action/takeoff_action.hpp"
#include "behaviour_trees/action/goto_action.hpp"
#include "behaviour_trees/action/land_action.hpp"
#include "behaviour_trees/action/arm_service.hpp"
#include "behaviour_trees/action/offboard_service.hpp"
#include "behaviour_trees/condition/is_flying_condition.hpp"
#include "behaviour_trees/decorator/wait_for_event.hpp"

#include "rclcpp/rclcpp.hpp"

// Groot connetion
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_node");

    node->declare_parameter("tree", "");
    std::string tree_description = node->get_parameter("tree").as_string();

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<as2_behaviour_tree::ArmService>("Arm");
    factory.registerNodeType<as2_behaviour_tree::DisarmService>("Disarm");
    factory.registerNodeType<as2_behaviour_tree::OffboardService>("Offboard");
    factory.registerNodeType<as2_behaviour_tree::TakeoffAction>("TakeOff");
    factory.registerNodeType<as2_behaviour_tree::GoToAction>("GoTo");
    factory.registerNodeType<as2_behaviour_tree::LandAction>("Land");
    factory.registerNodeType<as2_behaviour_tree::IsFlyingCondition>("IsFlying");
    factory.registerNodeType<as2_behaviour_tree::WaitForEvent>("WaitForEvent");

    BT::NodeConfiguration *config = new BT::NodeConfiguration();
    // Create the blackboard that will be shared by all of the nodes in the tree
    config->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config->blackboard->set<rclcpp::Node::SharedPtr>("node", node);
    config->blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10000));
    config->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));

    auto tree = factory.createTreeFromFile(tree_description, config->blackboard);

    // LOGGERS
    BT::StdCoutLogger logger_cout(tree);
    BT::PublisherZMQ groot_pub(tree);

    // to keep track of the number of ticks it took to reach a terminal result
    int ticks = 0;

    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    // BT loop execution rate
    rclcpp::WallRate loopRate(std::chrono::milliseconds(10));

    // main BT execution loop
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING)
    {
        result = tree.tickRoot();
        ticks++;
        loopRate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}