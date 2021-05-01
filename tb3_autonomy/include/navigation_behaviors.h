// Navigation behaviors for TurtleBot3

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Sets number of locations from list
class SetLocations : public BT::SyncActionNode
{
  public:
    SetLocations(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// Gets location from a queue of locations read from a list
class GetLocationFromQueue : public BT::SyncActionNode
{
  public:

    std::deque<std::string> location_queue_;

    GetLocationFromQueue(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// Go to a target location (wraps around `move_base`)
class GoToPose : public BT::StatefulActionNode
{
  public:

    MoveBaseClient client_;
    move_base_msgs::MoveBaseGoal goal_;

    GoToPose(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

};