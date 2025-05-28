#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/behavior_tree_status_change.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class SimplestNav2Bridge : public rclcpp::Node {
public:
    SimplestNav2Bridge() : Node("nav2_zmq_bridge") {
        // Subscribe to Nav2 BT status topics
        bt_log_sub_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
            "/behavior_tree_log", 10,
            std::bind(&SimplestNav2Bridge::bt_log_callback, this, std::placeholders::_1));
        
        bt_status_sub_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeStatusChange>(
            "/behavior_tree_status_change", 10,
            std::bind(&SimplestNav2Bridge::bt_status_callback, this, std::placeholders::_1));

        // Create a simple behavior tree using only built-in nodes
        setupBehaviorTree();
        
        // Create ZMQ publisher for Groot
        zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_);
        
        // Create timer to tick the tree
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SimplestNav2Bridge::tickTree, this));
        
        RCLCPP_INFO(this->get_logger(), "Simplest Nav2 ZMQ Bridge initialized");
        RCLCPP_INFO(this->get_logger(), "ZMQ publisher started on ports 1666/1667");
        RCLCPP_INFO(this->get_logger(), "Connect Groot: Monitor -> Connect via ZeroMQ");
        RCLCPP_INFO(this->get_logger(), "Monitoring Nav2 behavior tree status changes...");
    }

private:
    void setupBehaviorTree() {
        BT::BehaviorTreeFactory factory;
        
        // Use only built-in nodes - no custom node registration needed
        const std::string xml_text = R"(
        <root main_tree_to_execute="Nav2Visualization">
            <BehaviorTree ID="Nav2Visualization">
                <Sequence name="NavigationFlow">
                    <Fallback name="PlanningAttempt">
                        <Sequence name="NormalPlan">
                            <AlwaysSuccess name="ComputePath"/>
                            <AlwaysSuccess name="FollowPath"/>
                        </Sequence>
                        <Sequence name="RecoveryPlan">
                            <AlwaysSuccess name="ClearCostmaps"/>
                            <Fallback name="RecoveryBehaviors">
                                <AlwaysFailure name="SpinRecovery"/>
                                <AlwaysSuccess name="WaitRecovery"/>
                                <AlwaysFailure name="BackupRecovery"/>
                            </Fallback>
                        </Sequence>
                    </Fallback>
                </Sequence>
            </BehaviorTree>
        </root>
        )";
        
        tree_ = factory.createTreeFromText(xml_text);
        RCLCPP_INFO(this->get_logger(), "Behavior tree created successfully");
    }

    void bt_log_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received BT log with timestamp: %d", 
                    msg->timestamp.sec);
        has_received_bt_data_ = true;
        message_count_++;
    }
    
    void bt_status_callback(const nav2_msgs::msg::BehaviorTreeStatusChange::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "[Nav2 BT] %s: %s -> %s", 
                   msg->node_name.c_str(), 
                   msg->previous_status.c_str(), 
                   msg->current_status.c_str());
        
        has_received_bt_data_ = true;
        message_count_++;
        
        // Track what's happening in Nav2
        trackNav2Activity(msg->node_name, msg->current_status);
    }
    
    void trackNav2Activity(const std::string& node_name, const std::string& status) {
        // Just log the activity patterns we see
        if (status == "RUNNING") {
            RCLCPP_INFO(this->get_logger(), "🔄 Nav2 is executing: %s", node_name.c_str());
        } else if (status == "SUCCESS") {
            RCLCPP_INFO(this->get_logger(), "✅ Nav2 succeeded: %s", node_name.c_str());
        } else if (status == "FAILURE") {
            RCLCPP_INFO(this->get_logger(), "❌ Nav2 failed: %s", node_name.c_str());
        }
        
        // Update our understanding of Nav2 state
        if (node_name.find("compute_path") != std::string::npos) {
            planning_active_ = (status == "RUNNING");
        } else if (node_name.find("follow_path") != std::string::npos) {
            following_active_ = (status == "RUNNING");
        } else if (node_name.find("spin") != std::string::npos || 
                   node_name.find("backup") != std::string::npos || 
                   node_name.find("wait") != std::string::npos) {
            recovery_active_ = (status == "RUNNING");
        }
    }
    
    void tickTree() {
        if (has_received_bt_data_) {
            // Tick the tree to keep Groot updated
            tree_.tickRoot();
            
            // Provide status info
            if (message_count_ % 10 == 0) { // Every ~5 seconds
                RCLCPP_INFO(this->get_logger(), 
                    "📊 Nav2 Status - Planning: %s, Following: %s, Recovery: %s (Messages: %d)",
                    planning_active_ ? "ACTIVE" : "idle",
                    following_active_ ? "ACTIVE" : "idle", 
                    recovery_active_ ? "ACTIVE" : "idle",
                    message_count_);
            }
        } else {
            // Tick anyway to show tree structure in Groot
            tree_.tickRoot();
            
            // Warn about no data
            static auto last_warning = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (now - last_warning > std::chrono::seconds(15)) {
                RCLCPP_WARN(this->get_logger(), 
                    "No Nav2 behavior tree messages received yet. Send a navigation goal to see activity.");
                last_warning = now;
            }
        }
    }
    
    // Member variables
    BT::Tree tree_;
    std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_log_sub_;
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeStatusChange>::SharedPtr bt_status_sub_;
    
    bool has_received_bt_data_ = false;
    int message_count_ = 0;
    
    // Nav2 state tracking
    bool planning_active_ = false;
    bool following_active_ = false;
    bool recovery_active_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplestNav2Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}