#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/behavior_tree_status_change.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class Nav2ZmqBridge : public rclcpp::Node {
public:
    Nav2ZmqBridge() : Node("nav2_zmq_bridge") {
        // Subscribe to Nav2 BT status topics
        bt_log_sub_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
            "/behavior_tree_log", 10,
            std::bind(&Nav2ZmqBridge::bt_log_callback, this, std::placeholders::_1));
        
        bt_status_sub_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeStatusChange>(
            "/behavior_tree_status_change", 10,
            std::bind(&Nav2ZmqBridge::bt_status_callback, this, std::placeholders::_1));

        // Create a simple dummy tree for ZMQ publishing
        BT::BehaviorTreeFactory factory;
        
        // Create a minimal tree structure that represents Nav2's BT
        const std::string xml_text = R"(
        <root main_tree_to_execute="Nav2Tree">
            <BehaviorTree ID="Nav2Tree">
                <Sequence name="NavigateWithReplanning">
                    <Fallback name="PlannerFallback">
                        <Sequence name="PlanAndMove">
                            <Action ID="ComputePathToPose" name="ComputePathToPose"/>
                            <Action ID="FollowPath" name="FollowPath"/>
                        </Sequence>
                        <Sequence name="RecoveryActions">
                            <Action ID="ClearEntireCostmap" name="ClearLocalCostmap"/>
                            <Action ID="Spin" name="Spin"/>
                            <Action ID="BackUp" name="BackUp"/>
                        </Sequence>
                    </Fallback>
                </Sequence>
            </BehaviorTree>
        </root>
        )";
        
        // Create the tree
        tree_ = factory.createTreeFromText(xml_text);
        
        // Create ZMQ publisher for Groot
        zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_);
        
        // Create timer to tick the dummy tree (this will show structure in Groot)
        timer_ = this->create_wall_timer(
            1s, std::bind(&Nav2ZmqBridge::tick_dummy_tree, this));
        
        RCLCPP_INFO(this->get_logger(), "Nav2 ZMQ Bridge initialized");
        RCLCPP_INFO(this->get_logger(), "ZMQ publisher started on ports 1666/1667");
        RCLCPP_INFO(this->get_logger(), "Connect Groot using: Monitor -> Connect via ZeroMQ");
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 behavior tree messages...");
    }

private:
    void bt_log_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
        // Check what fields are actually available in the BehaviorTreeLog message
        // Since 'event' field doesn't exist, let's see what fields are available
        RCLCPP_DEBUG(this->get_logger(), "Received BT log message with timestamp: %ld", 
                    msg->timestamp.sec);
        
        // Process the behavior tree log and update our dummy tree status
        // This would require more complex parsing to map Nav2's actual BT to our dummy tree
        last_log_time_ = this->get_clock()->now();
        has_received_bt_data_ = true;
    }
    
    void bt_status_callback(const nav2_msgs::msg::BehaviorTreeStatusChange::SharedPtr msg) {
        // Fix the format strings - these are strings, not integers
        RCLCPP_INFO(this->get_logger(), "BT Status Change - Node: %s, Previous: %s, Current: %s", 
                   msg->node_name.c_str(), msg->previous_status.c_str(), msg->current_status.c_str());
        
        last_status_time_ = this->get_clock()->now();
        has_received_bt_data_ = true;
    }
    
    void tick_dummy_tree() {
        if (has_received_bt_data_) {
            RCLCPP_DEBUG(this->get_logger(), "Ticking dummy tree for Groot visualization...");
            tree_.tickRoot();
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                "No Nav2 BT messages received yet. Make sure Nav2 is running and sending navigation goals.");
        }
    }
    
    BT::Tree tree_;
    std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_log_sub_;
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeStatusChange>::SharedPtr bt_status_sub_;
    
    rclcpp::Time last_log_time_;
    rclcpp::Time last_status_time_;
    bool has_received_bt_data_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2ZmqBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}