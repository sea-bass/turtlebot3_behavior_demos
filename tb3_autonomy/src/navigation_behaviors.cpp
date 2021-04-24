#include "navigation_behaviors.h"

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "yaml-cpp/yaml.h"


GoToPose::GoToPose(const std::string& name, const std::vector<float>& pose) :
    BT::StatefulActionNode(name, {}), 
    client_("move_base", true),
    pose_(pose)
{
    client_.waitForServer();
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus GoToPose::onStart() {
    std::cout << "[" << this->name() << "] Sending goal..." << std::endl;
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.pose.position.x = pose_[0];
    goal_.target_pose.pose.position.y = pose_[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, pose_[2]);
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


// PORT

GoToPosePort::GoToPosePort(const std::string& name, const BT::NodeConfiguration& config) :
    BT::StatefulActionNode(name, config), 
    client_("move_base", true)
{
    client_.waitForServer();
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus GoToPosePort::onStart() {    
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

BT::NodeStatus GoToPosePort::onRunning() {
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

void GoToPosePort::onHalted() {};

BT::PortsList GoToPosePort::providedPorts() {
    return { BT::InputPort<std::vector<float>>("loc") };
}