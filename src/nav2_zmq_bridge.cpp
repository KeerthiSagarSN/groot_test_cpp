#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/behavior_tree_status_change.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class AdaptiveNav2Bridge : public rclcpp::Node {
public:
    AdaptiveNav2Bridge() : Node("nav2_zmq_bridge") {
        // Try multiple possible topic names for behavior tree status
        setupBehaviorTreeSubscriptions();
        
        // Also monitor action servers as backup
        setupActionSubscriptions();

        // Create behavior tree
        setupBehaviorTree();
        
        // Create ZMQ publisher for Groot
        zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_);
        
        // Create timer to update status
        timer_ = this->create_wall_timer(
            200ms, std::bind(&AdaptiveNav2Bridge::updateStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Adaptive Nav2 ZMQ Bridge initialized");
        RCLCPP_INFO(this->get_logger(), "ZMQ publisher started on ports 1666/1667");
        RCLCPP_INFO(this->get_logger(), "Monitoring multiple Nav2 data sources...");
        
        // Check what topics are available
        checkAvailableTopics();
    }

private:
    void setupBehaviorTreeSubscriptions() {
        // Try different possible topic names for BT status
        bt_status_sub1_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeStatusChange>(
            "/behavior_tree_status_change", 10,
            std::bind(&AdaptiveNav2Bridge::bt_status_callback, this, std::placeholders::_1));
        
        bt_status_sub2_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeStatusChange>(
            "/bt_navigator/behavior_tree_status_change", 10,
            std::bind(&AdaptiveNav2Bridge::bt_status_callback, this, std::placeholders::_1));
        
        bt_log_sub1_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
            "/behavior_tree_log", 10,
            std::bind(&AdaptiveNav2Bridge::bt_log_callback, this, std::placeholders::_1));
        
        bt_log_sub2_ = this->create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
            "/bt_navigator/behavior_tree_log", 10,
            std::bind(&AdaptiveNav2Bridge::bt_log_callback, this, std::placeholders::_1));
    }
    
    void setupActionSubscriptions() {
        // Monitor action servers - using exact topic names from your action list
        nav_action_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            std::bind(&AdaptiveNav2Bridge::nav_action_callback, this, std::placeholders::_1));
        
        compute_action_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/compute_path_to_pose/_action/status", 10,
            std::bind(&AdaptiveNav2Bridge::compute_action_callback, this, std::placeholders::_1));
        
        follow_action_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/follow_path/_action/status", 10,
            std::bind(&AdaptiveNav2Bridge::follow_action_callback, this, std::placeholders::_1));
        
        spin_action_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/spin/_action/status", 10,
            std::bind(&AdaptiveNav2Bridge::spin_action_callback, this, std::placeholders::_1));
        
        backup_action_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/backup/_action/status", 10,
            std::bind(&AdaptiveNav2Bridge::backup_action_callback, this, std::placeholders::_1));
        
        wait_action_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/wait/_action/status", 10,
            std::bind(&AdaptiveNav2Bridge::wait_action_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to action status topics for: navigate_to_pose, compute_path_to_pose, follow_path, spin, backup, wait");
    }

    void setupBehaviorTree() {
        BT::BehaviorTreeFactory factory;
        
        const std::string xml_text = R"(
        <root main_tree_to_execute="Nav2Monitor">
            <BehaviorTree ID="Nav2Monitor">
                <Sequence name="NavigationSequence">
                    <Fallback name="PlanningFallback">
                        <Sequence name="NormalNavigation">
                            <AlwaysSuccess name="ComputePathAction"/>
                            <AlwaysSuccess name="FollowPathAction"/>
                        </Sequence>
                        <Sequence name="RecoverySequence">
                            <AlwaysSuccess name="ClearCostmaps"/>
                            <Fallback name="RecoveryActions">
                                <AlwaysFailure name="SpinAction"/>
                                <AlwaysSuccess name="WaitAction"/>
                                <AlwaysFailure name="BackupAction"/>
                            </Fallback>
                        </Sequence>
                    </Fallback>
                </Sequence>
            </BehaviorTree>
        </root>
        )";
        
        tree_ = factory.createTreeFromText(xml_text);
        RCLCPP_INFO(this->get_logger(), "Adaptive monitoring tree created");
    }
    
    void checkAvailableTopics() {
        // This will help us understand what's actually available
        auto timer = this->create_wall_timer(5s, [this]() {
            RCLCPP_INFO(this->get_logger(), 
                "📊 Data Sources - BT Messages: %d, Action Messages: %d, Total: %d",
                bt_message_count_, action_message_count_, bt_message_count_ + action_message_count_);
            
            if (bt_message_count_ == 0 && action_message_count_ == 0) {
                RCLCPP_WARN(this->get_logger(), "No data received yet. Send a navigation goal to test.");
            }
        });
        
        // Cancel after first run
        this->create_wall_timer(6s, [timer]() { timer->cancel(); });
    }

    // Behavior Tree Callbacks
    void bt_status_callback(const nav2_msgs::msg::BehaviorTreeStatusChange::SharedPtr msg) {
        bt_message_count_++;
        has_bt_data_ = true;
        
        RCLCPP_INFO(this->get_logger(), "🌳 [BT] %s: %s -> %s", 
                   msg->node_name.c_str(), 
                   msg->previous_status.c_str(), 
                   msg->current_status.c_str());
        
        updateNavigationState("BT", msg->node_name, msg->current_status);
    }
    
    void bt_log_callback(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) {
        bt_message_count_++;
        has_bt_data_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Received BT log with timestamp: %d", msg->timestamp.sec);
    }
    
    // Action Callbacks
    void nav_action_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        processActionStatus("Navigate", msg);
    }
    
    void compute_action_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        processActionStatus("ComputePath", msg);
    }
    
    void follow_action_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        processActionStatus("FollowPath", msg);
    }
    
    void spin_action_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        processActionStatus("Spin", msg);
    }
    
    void backup_action_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        processActionStatus("Backup", msg);
    }
    
    void wait_action_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        processActionStatus("Wait", msg);
    }
    
    void processActionStatus(const std::string& action_name, const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        if (!msg->status_list.empty()) {
            action_message_count_++;
            has_action_data_ = true;
            
            for (const auto& status : msg->status_list) {
                if (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                    RCLCPP_INFO(this->get_logger(), "⚡ [ACTION] %s: EXECUTING", action_name.c_str());
                    updateNavigationState("ACTION", action_name, "RUNNING");
                } else if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "✅ [ACTION] %s: SUCCEEDED", action_name.c_str());
                    updateNavigationState("ACTION", action_name, "SUCCESS");
                } else if (status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
                    RCLCPP_INFO(this->get_logger(), "❌ [ACTION] %s: ABORTED", action_name.c_str());
                    updateNavigationState("ACTION", action_name, "FAILURE");
                }
            }
        }
    }
    
    void updateNavigationState(const std::string& source, const std::string& component, const std::string& status) {
        // Track what's happening for status reporting
        if (component.find("compute") != std::string::npos || component.find("ComputePath") != std::string::npos) {
            compute_path_active_ = (status == "RUNNING" || status == "EXECUTING");
        } else if (component.find("follow") != std::string::npos || component.find("FollowPath") != std::string::npos) {
            follow_path_active_ = (status == "RUNNING" || status == "EXECUTING");
        } else if (component.find("spin") != std::string::npos || component.find("Spin") != std::string::npos) {
            spin_active_ = (status == "RUNNING" || status == "EXECUTING");
        } else if (component.find("wait") != std::string::npos || component.find("Wait") != std::string::npos) {
            wait_active_ = (status == "RUNNING" || status == "EXECUTING");
        } else if (component.find("backup") != std::string::npos || component.find("BackUp") != std::string::npos) {
            backup_active_ = (status == "RUNNING" || status == "EXECUTING");
        }
    }
    
    void updateStatus() {
        bool has_any_data = has_bt_data_ || has_action_data_;
        
        if (has_any_data) {
            // Regular status update
            static int counter = 0;
            if (++counter % 25 == 0) { // Every ~5 seconds
                RCLCPP_INFO(this->get_logger(), 
                    "📊 Nav2 Activity - Compute: %s, Follow: %s, Spin: %s, Wait: %s, Backup: %s",
                    compute_path_active_ ? "ACTIVE" : "idle",
                    follow_path_active_ ? "ACTIVE" : "idle",
                    spin_active_ ? "ACTIVE" : "idle",
                    wait_active_ ? "ACTIVE" : "idle",
                    backup_active_ ? "ACTIVE" : "idle");
                    
                RCLCPP_INFO(this->get_logger(), 
                    "📡 Data Sources - BT: %s (%d msgs), Actions: %s (%d msgs)",
                    has_bt_data_ ? "CONNECTED" : "no data",
                    bt_message_count_,
                    has_action_data_ ? "CONNECTED" : "no data", 
                    action_message_count_);
            }
        } else {
            static auto last_warning = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (now - last_warning > std::chrono::seconds(15)) {
                RCLCPP_WARN(this->get_logger(), 
                    "No Nav2 data received. Check: 1) Nav2 is running, 2) Send navigation goal, 3) BT logging enabled");
                last_warning = now;
            }
        }
        
        // Always tick tree for Groot
        tree_.tickRoot();
    }
    
    // Member variables
    BT::Tree tree_;
    std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // BT Subscriptions (multiple topic names)
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeStatusChange>::SharedPtr bt_status_sub1_;
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeStatusChange>::SharedPtr bt_status_sub2_;
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_log_sub1_;
    rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr bt_log_sub2_;
    
    // Action Subscriptions
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_action_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr compute_action_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr follow_action_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr spin_action_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr backup_action_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr wait_action_sub_;
    
    // Status tracking
    bool has_bt_data_ = false;
    bool has_action_data_ = false;
    int bt_message_count_ = 0;
    int action_message_count_ = 0;
    
    // Activity states
    bool compute_path_active_ = false;
    bool follow_path_active_ = false;
    bool spin_active_ = false;
    bool wait_active_ = false;
    bool backup_active_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdaptiveNav2Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}