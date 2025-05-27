#ifndef GROOT_TEST_CPP__BT_NODES_HPP_
#define GROOT_TEST_CPP__BT_NODES_HPP_

#include <string>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace groot_test {

// Simple action node that succeeds
class SayHello : public BT::SyncActionNode {
public:
  SayHello(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config) {
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("message", "Message to print")
    };
  }

  BT::NodeStatus tick() override {
    std::string message;
    if (!getInput("message", message)) {
      message = "Hello, world!";
    }
    
    RCLCPP_INFO(rclcpp::get_logger("bt_node"), "%s", message.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

// Action node that toggles between running and success
// Using StatefulActionNode instead of SyncActionNode to allow RUNNING state
class ToggleStatus : public BT::StatefulActionNode {
public:
  ToggleStatus(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::StatefulActionNode(name, config), counter_(0) {
  }

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus onStart() override {
    counter_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    counter_++;
    if (counter_ % 3 == 0) {
      RCLCPP_INFO(rclcpp::get_logger("bt_node"), "Node %s returns SUCCESS", name().c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("bt_node"), "Node %s returns RUNNING", name().c_str());
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override {
    RCLCPP_INFO(rclcpp::get_logger("bt_node"), "Node %s halted", name().c_str());
  }

private:
  int counter_;
};

// Action node that always fails
class AlwaysFail : public BT::SyncActionNode {
public:
  AlwaysFail(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config) {
  }

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    RCLCPP_INFO(rclcpp::get_logger("bt_node"), "Node %s returns FAILURE", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
};

} // namespace groot_test

#endif // GROOT_TEST_CPP__BT_NODES_HPP_
