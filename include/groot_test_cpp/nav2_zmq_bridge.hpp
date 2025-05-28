#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace nav2_groot_test {

class NavigateToPose : public BT::SyncActionNode {
public:
    NavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), counter_(0) {
        node_ = rclcpp::Node::make_shared("nav_to_pose_bt_node");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("goal_x"),
            BT::InputPort<double>("goal_y"),
            BT::InputPort<double>("goal_theta")
        };
    }

    BT::NodeStatus tick() override {
        // Get goal parameters
        double x = getInput<double>("goal_x").value();
        double y = getInput<double>("goal_y").value();
        double theta = getInput<double>("goal_theta").value();
        
        RCLCPP_INFO(node_->get_logger(), "NavigateToPose: Going to (%.2f, %.2f, %.2f)", x, y, theta);
        
        // Simulate navigation behavior
        counter_++;
        
        if (counter_ < 5) {
            return BT::NodeStatus::RUNNING;
        } else {
            counter_ = 0;
            return BT::NodeStatus::SUCCESS;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    int counter_;
};

class Spin : public BT::SyncActionNode {
public:
    Spin(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), counter_(0) {
        node_ = rclcpp::Node::make_shared("spin_bt_node");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("spin_dist")
        };
    }

    BT::NodeStatus tick() override {
        double spin_dist = getInput<double>("spin_dist").value();
        RCLCPP_INFO(node_->get_logger(), "Spin: Spinning %.2f radians", spin_dist);
        
        // Simulate spin behavior
        counter_++;
        
        if (counter_ < 3) {
            return BT::NodeStatus::RUNNING;
        } else {
            counter_ = 0;
            return BT::NodeStatus::SUCCESS;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    int counter_;
};

class Wait : public BT::SyncActionNode {
public:
    Wait(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), counter_(0) {
        node_ = rclcpp::Node::make_shared("wait_bt_node");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("wait_duration")
        };
    }

    BT::NodeStatus tick() override {
        double duration = getInput<double>("wait_duration").value();
        RCLCPP_INFO(node_->get_logger(), "Wait: Waiting for %.2f seconds", duration);
        
        // Simulate wait behavior
        counter_++;
        
        if (counter_ < 2) {
            return BT::NodeStatus::RUNNING;
        } else {
            counter_ = 0;
            return BT::NodeStatus::SUCCESS;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    int counter_;
};

class BackUp : public BT::SyncActionNode {
public:
    BackUp(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), counter_(0) {
        node_ = rclcpp::Node::make_shared("backup_bt_node");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("backup_dist")
        };
    }

    BT::NodeStatus tick() override {
        double backup_dist = getInput<double>("backup_dist").value();
        RCLCPP_INFO(node_->get_logger(), "BackUp: Backing up %.2f meters", backup_dist);
        
        // Simulate backup behavior
        counter_++;
        
        if (counter_ < 2) {
            return BT::NodeStatus::RUNNING;
        } else {
            counter_ = 0;
            return BT::NodeStatus::SUCCESS;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    int counter_;
};

} // namespace nav2_groot_test