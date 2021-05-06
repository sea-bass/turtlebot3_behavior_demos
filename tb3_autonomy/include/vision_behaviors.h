// Vision behaviors for TurtleBot3

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

// HSV Thresholding parameters
// The convention is {H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX}
typedef std::map<std::string, std::vector<int>> ColorThresholdMap;
const ColorThresholdMap hsv_threshold_dict = {
    {"red", {0, 30, 220, 255, 0, 255}},
    {"green", {40, 80, 220, 255, 0, 255}},
    {"blue", {100, 140, 220, 255, 0, 255}},
};

// Look for an object of a particular color
class LookForObject : public BT::ConditionNode
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