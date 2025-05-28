#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <chrono>
#include <memory>
#include "groot_test_cpp/nav2_bt_nodes.hpp"

using namespace std::chrono_literals;

class Nav2GrootTestNode : public rclcpp::Node {
public:
    Nav2GrootTestNode() : Node("nav2_groot_test_node") {
        // Create BT factory and register custom Nav2 nodes
        BT::BehaviorTreeFactory factory;
        
        // Register custom Nav2 node types
        factory.registerNodeType<nav2_groot_test::NavigateToPose>("NavigateToPose");
        factory.registerNodeType<nav2_groot_test::Spin>("Spin");
        factory.registerNodeType<nav2_groot_test::Wait>("Wait");
        factory.registerNodeType<nav2_groot_test::BackUp>("BackUp");
        
        // Define a simple Nav2 behavior tree using XML
        const std::string xml_text = R"(
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <Sequence name="nav2_sequence">
                    <NavigateToPose name="nav_to_goal" 
                                   goal_x="2.0" 
                                   goal_y="1.0" 
                                   goal_theta="0.0"/>
                    <Fallback name="recovery_fallback">
                        <Wait name="wait_node" wait_duration="2.0"/>
                        <Sequence name="recovery_sequence">
                            <BackUp name="backup_node" backup_dist="0.5"/>
                            <Spin name="spin_node" spin_dist="1.57"/>
                        </Sequence>
                    </Fallback>
                </Sequence>
            </BehaviorTree>
        </root>
        )";
        
        // Create the tree
        tree_ = factory.createTreeFromText(xml_text);
        
        // Create a ZMQ publisher for Groot
        // NOTE: Using the BT.CPP 3.8 API which is slightly different
        zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_);
        
        // Create a timer to tick the tree periodically
        timer_ = this->create_wall_timer(
            1s, std::bind(&Nav2GrootTestNode::tick_tree, this));
        
        RCLCPP_INFO(this->get_logger(), "Nav2 Groot test node initialized");
        RCLCPP_INFO(this->get_logger(), "ZMQ publisher started on ports 1666/1667");
        RCLCPP_INFO(this->get_logger(), "Connect Groot using: Monitor -> Connect via ZeroMQ");
    }

private:
    void tick_tree() {
        RCLCPP_INFO(this->get_logger(), "Ticking the Nav2 tree...");
        tree_.tickRoot();
    }
    
    BT::Tree tree_;
    std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2GrootTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}