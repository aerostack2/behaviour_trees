#include <chrono>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "basic_behaviours.hpp"

#include "rclcpp/rclcpp.hpp"

// Groot connetion
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_node");

    node->declare_parameter("tree");
    std::string tree_description = node->get_parameter("tree").as_string();

    // TODO --> get namespace
    
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder builder_tk = [](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<as2_behaviour_tree::TakeoffAction>(name, config, "/drone_sim_0/TakeOffBehaviour");
    };

    factory.registerBuilder<as2_behaviour_tree::TakeoffAction>("TakeOff", builder_tk);

    // TODO --> new nodes to arm offboard

    // TODO --> All ActionNode to NodeBuilders
    // factory.registerNodeType<as2_behaviour_tree::TakeoffAction>("TakeOff");
    factory.registerNodeType<as2_behaviour_tree::GoToAction>("GoTo");
    factory.registerNodeType<as2_behaviour_tree::LandAction>("Land");

    BT::NodeConfiguration* config = new BT::NodeConfiguration();
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
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
        result = tree.tickRoot();
        ticks++;
        loopRate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}