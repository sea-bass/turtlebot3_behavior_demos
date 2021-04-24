#include "behaviortree_cpp_v3/behavior_tree.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoToPose : public BT::StatefulActionNode
{
  public:

    MoveBaseClient client_;
    move_base_msgs::MoveBaseGoal goal_;
    std::vector<float> pose_;

    GoToPose(const std::string& name, const std::vector<float>& pose={1.0, 0.0, 0.0});

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

};

class GoToPosePort : public BT::StatefulActionNode
{
  public:

    MoveBaseClient client_;
    move_base_msgs::MoveBaseGoal goal_;

    GoToPosePort(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

};