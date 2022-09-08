// Navigation Related Behaviors

#include "navigation_behaviors.h"

#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "yaml-cpp/yaml.h"


// SETLOCATIONS
// Gets a list of locations from a YAML file and ensures they are not empty
SetLocations::SetLocations(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config)
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus SetLocations::tick()
{
    std::string yaml_file = "/overlay_ws/src/tb3_worlds/maps/sim_house_locations.yaml";
    // ros::param::get("location_file", yaml_file);
    YAML::Node locations = YAML::LoadFile(yaml_file);
    int num_locs = locations.size();
    if (num_locs == 0) {
        std::cout << "[" << this->name() << "] No locations found." << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    setOutput("num_locs", num_locs);
    std::cout << "[" << this->name() << "] Found " << num_locs << " locations." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetLocations::providedPorts()
{
    return { BT::OutputPort<int>("num_locs") };
}


// GETLOCATIONFROMQUEUE
// Gets a location name from a queue of locations to visit.
// If the queue is empty, this behavior fails.
GetLocationFromQueue::GetLocationFromQueue(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config)
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

void GetLocationFromQueue::init(rclcpp::Node::SharedPtr node_ptr) {
    node_ptr_ = node_ptr;

    // Get the locations from the file specified in the ROS parameter, put them
    // into the location queue, and shuffle it.
    const std::string location_file = 
        node_ptr_->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(location_file);
    for(YAML::const_iterator it=locations.begin(); it!=locations.end(); ++it) {
        location_queue_.push_front(it->first.as<std::string>());
    }
    std::random_device rd;
    std::mt19937 rng(rd());
    std::shuffle(location_queue_.begin(), location_queue_.end(), rng);
}

BT::NodeStatus GetLocationFromQueue::tick()
{
    if (location_queue_.empty()) {
        std::cout << "[" << this->name() << "] No more locations!" << std::endl;
        return BT::NodeStatus::FAILURE;
    } else {
        std::string tgt_loc = location_queue_.front();
        setOutput("target_location", tgt_loc);
        location_queue_.pop_front();
        std::cout << "[" << this->name() << "] Targeting location: " << tgt_loc << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
}

BT::PortsList GetLocationFromQueue::providedPorts()
{
    return { BT::OutputPort<std::string>("target_location") };
}


// GOTOPOSE
// Wrapper behavior around the `navigate_to_pose` action client,
// whose status reflects the status of the ROS action.
GoToPose::GoToPose(const std::string& name, const BT::NodeConfiguration& config) :
    BT::StatefulActionNode(name, config) {}

void GoToPose::init(rclcpp::Node::SharedPtr node_ptr) {
    node_ptr_ = node_ptr;
}

BT::NodeStatus GoToPose::onStart() {
    // Validate that a node exists
    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Read the YAML file
    BT::Optional<std::string> loc = getInput<std::string>("loc");
    const std::string location_file = 
        node_ptr_->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(location_file);
    std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

    // Set up the action client
    using namespace std::placeholders;
    auto send_goal_options = 
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&GoToPose::result_callback, this, _1);
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      node_ptr_, "/navigate_to_pose");

    // Package up the node
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Send the navigation action goal.
    done_flag_ = false;
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::cout << "[" << this->name() << "] Sent goal message" << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning() {
    if (done_flag_) {
        std::cout << "[" << this->name() << "] Goal reached" << std::endl;
        return BT::NodeStatus::SUCCESS;   
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

BT::PortsList GoToPose::providedPorts() {
    return { BT::InputPort<std::string>("loc") };
}

void GoToPose::result_callback(const GoalHandleNav::WrappedResult & result) {
    // If there is a result, we consider navigation completed.
    // bt_navigator only sends an empty message without status because reasons.
    if (result.result) {
        done_flag_ = true;
    }
}
