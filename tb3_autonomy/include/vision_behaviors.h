#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

class LookForObject : public BT::SyncActionNode
{
  public:

    ros::NodeHandle nh_;
    bool received_image_;
    sensor_msgs::ImageConstPtr latest_image_;

    LookForObject(const std::string& name);
    void init(const ros::NodeHandle& nh);
    BT::NodeStatus tick() override;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};