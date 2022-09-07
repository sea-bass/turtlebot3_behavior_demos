/*
 * Main behavior node for TurtleBot3. 
 */

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "yaml-cpp/yaml.h"

#include "navigation_behaviors.h"
#include "vision_behaviors.h"

using namespace std::chrono_literals;

// // "Naive" implementation where there is a hard-coded sequence for each location.
// // Here, there is a top-level fallback node that tries locations sequentially.
// // As you can see, this does not automatically scale with number of locations.
// static const char* xml_text_naive = R"(
//  <root main_tree_to_execute = "MainTree" >
//      <BehaviorTree ID="MainTree">
//         <Fallback name="root">
//             <Sequence name="search_location1">
//                 <GoToPose       name="go_to_location1"  loc="location1" />
//                 <LookForObject  name="look_in_location1"/>
//             </Sequence>
//             <Sequence name="search_location2">
//                 <GoToPose       name="go_to_location2" loc="location2"/>
//                 <LookForObject  name="look_in_location2"/>
//             </Sequence>
//             <Sequence name="search_location3">
//                 <GoToPose       name="go_to_location3" loc="location3"/>
//                 <LookForObject  name="look_in_location3"/>
//             </Sequence>
//             <Sequence name="search_location3">
//                 <GoToPose       name="go_to_location4" loc="location4"/>
//                 <LookForObject  name="look_in_location4"/>
//             </Sequence>
//         </Fallback>
//      </BehaviorTree>
//  </root>
//  )";

// // A better implementation which uses a queue of location names to iterate through 
// // visiting locations regardless of number of locations.
// static const char* xml_text_queue = R"(
// <root main_tree_to_execute = "MainTree" >
//     <BehaviorTree ID="MainTree">
//     <Sequence name="main_loop">
//         <SetLocations                   name="set_locations" num_locs="{num_locs}"/>
//         <RetryUntilSuccessful           num_attempts="{num_locs}">
//             <Sequence                   name="search_location">
//                 <GetLocationFromQueue   name="get_loc"      target_location="{target_location}"/>   
//                 <GoToPose               name="go_to_loc"    loc="{target_location}"/>
//                 <LookForObject          name="look_for_obj" />
//             </Sequence>   
//         </RetryUntilSuccessful>
//     </Sequence>
//     </BehaviorTree>
// </root>
// )";

const std::string bt_xml_dir = 
    ament_index_cpp::get_package_share_directory("tb3_autonomy") + "/bt_xml";
const std::string tb3_worlds_share_dir = 
    ament_index_cpp::get_package_share_directory("tb3_worlds");
const std::string default_location_file =
    tb3_worlds_share_dir + "/maps/sim_house_locations.yaml";


class AutonomyNode : public rclcpp::Node {
    public:
        AutonomyNode() : Node("autonomy_node") {
            // Read the location file
            this->declare_parameter<std::string>(
                "location_file", default_location_file);
            const std::string location_file = 
                this->get_parameter("location_file").as_string();
            RCLCPP_INFO(this->get_logger(),
                "Using location file %s", location_file.c_str());
            YAML::Node locations = YAML::LoadFile(location_file);
            std::vector<std::string> loc_names;
            for(YAML::const_iterator it=locations.begin(); it!=locations.end(); ++it) {
                loc_names.push_back(it->first.as<std::string>());
            }
            std::srand(42);  // The answer
            std::random_shuffle(loc_names.begin(), loc_names.end());

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
            factory.registerNodeType<GoToPose>("GoToPose");
            factory.registerNodeType<SetLocations>("SetLocations");
            factory.registerNodeType<GetLocationFromQueue>("GetLocationFromQueue");
            factory.registerNodeType<LookForObject>("LookForObject");
            std::string tree_file;
            if (enable_vision_) {
                if (tree_type_ == "queue") {
                    tree_file = "tree_queue.xml";
                } else {
                    tree_file = "tree_naive.xml";
                }
            } else {
                if (tree_type_ == "queue") {
                    tree_file = "nav_tree_queue.xml";
                } else {
                    tree_file = "nav_tree_naive.xml";
                }
            }
            tree_ = factory.createTreeFromFile(bt_xml_dir + "/" + tree_file);

            // Inject a pointer to node to initalize behaviors that need it.
            for (auto &node : tree_.nodes) {
                if (auto node_ptr = dynamic_cast<GoToPose*>(node.get())) {
                    node_ptr->init(shared_from_this());
                } else if (auto node_ptr = dynamic_cast<LookForObject*>(node.get())) {
                    node_ptr->init(shared_from_this());
                }
            }
            
            // Set up logging to monitor the tree in Groot.
            // Default ports (1666/1667) are used by Nav2 BT.
            publisher_zmq_ptr_ = std::make_unique<BT::PublisherZMQ>(
                tree_, 25, 1668, 1669);
        }

        void update_behavior_tree() {
            // Tick the behavior tree.
            BT::NodeStatus tree_status = tree_.tickRoot();
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
