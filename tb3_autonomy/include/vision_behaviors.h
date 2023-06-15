// Vision behaviors for TurtleBot3

#include <map>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "behaviortree_cpp/behavior_tree.h"

// HSV Thresholding parameters
// The convention is {H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX}
typedef std::map<std::string, std::vector<int>> ColorThresholdMap;
const ColorThresholdMap hsv_threshold_dict = {
    {"red", {160, 180, 220, 255, 0, 255}},
    {"green", {40, 90, 220, 255, 0, 255}},
    {"blue", {100, 150, 220, 255, 0, 255}},
};

// Look for an object of a particular color
class LookForObject : public BT::StatefulActionNode
{
  public:

    rclcpp::Node::SharedPtr node_ptr_;
    bool received_image_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_image_ptr_;
    image_transport::Subscriber image_sub_;

    // Method overrides
    LookForObject(const std::string& name, const BT::NodeConfig& config,
                  rclcpp::Node::SharedPtr node_ptr);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts() { return {}; };

    // Image subscriber callback
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
};
