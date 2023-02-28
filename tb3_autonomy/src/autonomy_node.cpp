/*
 * Main behavior node for TurtleBot3. 
 */

#include <chrono>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include "yaml-cpp/yaml.h"

#include "navigation_behaviors.h"
#include "vision_behaviors.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir = 
    ament_index_cpp::get_package_share_directory("tb3_autonomy") + "/bt_xml";
const std::string tb3_worlds_share_dir = 
    ament_index_cpp::get_package_share_directory("tb3_worlds");
const std::string default_location_file =
    tb3_worlds_share_dir + "/maps/sim_house_locations.yaml";


// Helper to register behaviors that accept a pointer to a ROS node.
template <class NodeBehaviorT> 
void registerRosNodeType(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         rclcpp::Node::SharedPtr node_ptr) {
    BT::NodeBuilder builder = [=](const std::string& name,
                                  const BT::NodeConfig& config) {
        return std::make_unique<NodeBehaviorT>(name, config, node_ptr);
    };
    factory.registerBuilder<NodeBehaviorT>(registration_ID, builder);
}


class AutonomyNode : public rclcpp::Node {
    public:
        AutonomyNode() : Node("autonomy_node") {
            // Read the location file and shuffle it
            this->declare_parameter<std::string>(
                "location_file", default_location_file);
            location_file_ = 
                this->get_parameter("location_file").as_string();
            RCLCPP_INFO(this->get_logger(),
                "Using location file %s", location_file_.c_str());

            // Declare and get the other node parameters.
            this->declare_parameter<std::string>("tree_type", "naive");
            tree_type_ = this->get_parameter("tree_type").as_string();
            this->declare_parameter<bool>("enable_vision", true);
            enable_vision_ = this->get_parameter("enable_vision").as_bool();
            this->declare_parameter<std::string>("target_color", "blue");
            target_color_ = this->get_parameter("target_color").as_string();
            if (enable_vision_) {
                RCLCPP_INFO(this->get_logger(), "Searching for target color %s",
                    target_color_.c_str());
            }
        }

        void execute() {
            // Build and initialize the behavior tree based on parameters.
            create_behavior_tree();

            // Create a timer to tick the behavior tree.
            const auto timer_period = 500ms;
            timer_ = this->create_wall_timer(
                timer_period,
                std::bind(&AutonomyNode::update_behavior_tree, this));

            rclcpp::spin(shared_from_this());
            rclcpp::shutdown();
        }

        void create_behavior_tree() {
            // Build a behavior tree from XML and set it up for logging
            BT::BehaviorTreeFactory factory;
            factory.registerNodeType<SetLocations>("SetLocations");
            factory.registerNodeType<GetLocationFromQueue>("GetLocationFromQueue");
            registerRosNodeType<GoToPose>(
                factory, "GoToPose", shared_from_this());
            registerRosNodeType<LookForObject>(
                factory, "LookForObject", shared_from_this());
            
            const std::string tree_file = (enable_vision_ ? std::string{} : "nav_") + "tree_" + tree_type_ + ".xml";
            auto blackboard = BT::Blackboard::create();
            blackboard->set<std::string>("location_file", location_file_);
            tree_ = factory.createTreeFromFile(bt_xml_dir + "/" + tree_file, blackboard);
            
            // Set up logging to monitor the tree in Groot.
            // Default ports (1666/1667) are used by Nav2 BT.
            publisher_zmq_ptr_ = std::make_unique<BT::PublisherZMQ>(
                tree_, 25, 1668, 1669);
        }

        void update_behavior_tree() {
            // Tick the behavior tree.
            BT::NodeStatus tree_status = tree_.tickOnce();
            if (tree_status == BT::NodeStatus::RUNNING) {
                return;
            }
            // Cancel the timer if we hit a terminal state.
            if (tree_status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
                timer_->cancel();
            } else if (tree_status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
                timer_->cancel();
            }
        }

        // Member variables.
        std::string tree_type_;
        std::string location_file_;
        bool enable_vision_;
        std::string target_color_;
        rclcpp::TimerBase::SharedPtr timer_;
        BT::Tree tree_;
        std::unique_ptr<BT::PublisherZMQ> publisher_zmq_ptr_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomyNode>();
    node->execute();
    return 0;
}
