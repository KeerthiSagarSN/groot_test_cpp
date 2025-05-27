#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <chrono>
#include <memory>

#include "groot_test_cpp/bt_nodes.hpp"

using namespace std::chrono_literals;

class GrootTestNode : public rclcpp::Node {
public:
  GrootTestNode() : Node("groot_test_node") {
    // Create BT factory and register custom nodes
    BT::BehaviorTreeFactory factory;
    
    // Register custom node types
    factory.registerNodeType<groot_test::SayHello>("SayHello");
    factory.registerNodeType<groot_test::ToggleStatus>("ToggleStatus");
    factory.registerNodeType<groot_test::AlwaysFail>("AlwaysFail");
    
    // Define a simple behavior tree using XML
    const std::string xml_text = R"(
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <SayHello name="hello_node" message="Hello from Groot test!"/>
                <Fallback name="fallback_node">
                    <AlwaysFail name="failing_node"/>
                    <ToggleStatus name="toggle_node"/>
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
      1s, std::bind(&GrootTestNode::tick_tree, this));
    
    RCLCPP_INFO(this->get_logger(), "Groot test node initialized");
    RCLCPP_INFO(this->get_logger(), "ZMQ publisher started on ports 1666/1667");
    RCLCPP_INFO(this->get_logger(), "Connect Groot using: Monitor -> Connect via ZeroMQ");
  }
  
private:
  void tick_tree() {
    RCLCPP_INFO(this->get_logger(), "Ticking the tree...");
    tree_.tickRoot();
  }
  
  BT::Tree tree_;
  std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GrootTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
