// Navigation Related Behaviors

#include "navigation_behaviors.h"

#include <ros/ros.h>
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
    std::string yaml_file;
    ros::param::get("location_file", yaml_file);
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
    std::string yaml_file;
    ros::param::get("location_file", yaml_file);
    YAML::Node locations = YAML::LoadFile(yaml_file);
    for(YAML::const_iterator it=locations.begin(); it!=locations.end(); ++it) {
        location_queue_.push_front(it->first.as<std::string>());
    }
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
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
// Wrapper behavior around a `move_base` action client, whose status
// reflects the status of the ROS action.
GoToPose::GoToPose(const std::string& name, const BT::NodeConfiguration& config) :
    BT::StatefulActionNode(name, config), 
    client_("move_base", true)
{
    client_.waitForServer();
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus GoToPose::onStart() {    
    BT::Optional<std::string> loc = getInput<std::string>("loc");
    std::string yaml_file;
    ros::param::get("location_file", yaml_file);
    YAML::Node locations = YAML::LoadFile(yaml_file);
    std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

    std::cout << "[" << this->name() << "] Sending goal..." << std::endl;
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.pose.position.x = pose[0];
    goal_.target_pose.pose.position.y = pose[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    q.normalize();
    goal_.target_pose.pose.orientation = tf2::toMsg(q);
    client_.sendGoal(goal_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning() {
    actionlib::SimpleClientGoalState state = client_.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        std::cout << "[" << this->name() << "] Goal reached" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
        return BT::NodeStatus::RUNNING;
    } else {
        std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void GoToPose::onHalted() {};

BT::PortsList GoToPose::providedPorts() {
    return { BT::InputPort<std::string>("loc") };
}
