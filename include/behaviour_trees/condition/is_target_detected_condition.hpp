#ifndef IS_TARGET_DETECTED_CONDITION_HPP
#define IS_TARGET_DETECTED_CONDITION_HPP

#include <string>   

#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "as2_core/names/topics.hpp"

namespace as2_behaviour_tree
{
    class IsTargetDetectedCondition : public BT::ConditionNode
    {
    public:
        IsTargetDetectedCondition(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
            : BT::ConditionNode(xml_tag_name, conf)
        {
            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
            callback_group_ = node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive,
                false);
            callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

            getInput("topic_name", detection_topic_name_);
            getInput("dist_threshold", threshold_);

            rclcpp::SubscriptionOptions sub_option;
            sub_option.callback_group = callback_group_;

            odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                as2_names::topics::self_localization::odom,
                as2_names::topics::self_localization::qos,
                std::bind(&IsTargetDetectedCondition::odomCallback, this, std::placeholders::_1),
                sub_option
            );

            detection_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                detection_topic_name_,
                rclcpp::SystemDefaultsQoS(),
                std::bind(&IsTargetDetectedCondition::detectionCallback, this, std::placeholders::_1),
                sub_option);
        }

        IsTargetDetectedCondition() = delete;

        BT::NodeStatus tick()
        {
            callback_group_executor_.spin_some();
            if (is_target_) {

                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>("topic_name"), 
                    BT::InputPort<double>("dist_threshold"),
                    BT::OutputPort<std::string>("pose")};
        }

    private:
        void detectionCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            float dist = std::sqrt(std::pow(abs(msg->pose.position.x) - abs(this->current_pose_x_), 2.0) +
                                   std::pow(abs(msg->pose.position.y) - abs(this->current_pose_y_), 2.0) +
                                   std::pow(abs(msg->pose.position.z) - abs(this->current_pose_z_), 2.0));

            RCLCPP_DEBUG(this->node_->get_logger(), "%f", dist);
            if (dist < threshold_)
            {
                // geometry_msgs::msg::Pose rel_pose;
                // rel_pose.position.x = msg->pose.position.x - this->current_pose_x_;
                // rel_pose.position.y = msg->pose.position.y - this->current_pose_y_;
                // rel_pose.position.z = msg->pose.position.z - this->current_pose_z_;
                std::string output = std::to_string(msg->pose.position.x) + ";" 
                                   + std::to_string(msg->pose.position.y) + ";"
                                   + std::to_string(msg->pose.position.z);
                setOutput("pose", output);
                is_target_ = true;
            }
        }

        void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->current_pose_x_ = msg->pose.pose.position.x;
            this->current_pose_y_ = msg->pose.pose.position.y;
            this->current_pose_z_ = msg->pose.pose.position.z;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
        std::string detection_topic_name_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub_;
        double threshold_;
        bool is_target_ = false;

        std::atomic<float> current_pose_x_;
        std::atomic<float> current_pose_y_;
        std::atomic<float> current_pose_z_;
    };

} // namespace as2_behaviour_tree

#endif // IS_FLYING_CONDITION_HPP