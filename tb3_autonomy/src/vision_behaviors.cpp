// Vision related behaviors

#include "vision_behaviors.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <unistd.h>

using std::placeholders::_1;

// LOOKFOROBJECT
// Looks for an object of a certain color, specified by a parameter
LookForObject::LookForObject(const std::string& name,
                             const BT::NodeConfig& config,
                             rclcpp::Node::SharedPtr node_ptr) :
    BT::StatefulActionNode(name, config), node_ptr_{node_ptr}
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus LookForObject::onStart() {
    received_image_ = false;
    image_sub_ = image_transport::create_subscription(
        node_ptr_.get(), "/camera/image_raw",
        std::bind(&LookForObject::image_callback, this, _1),
        "raw", rmw_qos_profile_sensor_data);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LookForObject::onRunning()
{
    // std::string target_color = "blue";
    std::string target_color =
        node_ptr_->get_parameter("target_color").as_string();
    std::cout << "[" << this->name() << "] Looking for " << target_color << " object" << std::endl;
    
    // Wait to receive an image
    // TODO Add timeout?
    if (!received_image_) {
        // std::cout << "[" << this->name() << "] Waiting for image" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    // Convert to HSV and threshold
    std::vector<int> th = hsv_threshold_dict.at(target_color);
    cv::Mat img, img_hsv, img_threshold, img_keypoints;
    img = cv_bridge::toCvShare(latest_image_ptr_, "bgr8")->image;
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, 
        cv::Scalar(th[0], th[2], th[4]), 
        cv::Scalar(th[1], th[3], th[5]), img_threshold);

    // Do blob detection
    cv::SimpleBlobDetector::Params params;
    params.minArea = 100;
    params.maxArea = 100000;
    params.filterByArea = true;
    params.filterByColor = false;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.thresholdStep = 50;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img_threshold, keypoints);
    cv::drawKeypoints(img, keypoints, img_keypoints, 
        cv::Scalar(255,0,0), 
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display the image
    cv::namedWindow("Image");
    cv::imshow("Image", img_keypoints);
    cv::waitKey(2000);

    if (keypoints.size() > 0) {
        std::cout << "[" << this->name() << "] Found object" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << "[" << this->name() << "] No object detected" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void LookForObject::onHalted() {
    image_sub_.shutdown();
    received_image_ = false;
}

void LookForObject::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    latest_image_ptr_ = msg;
    received_image_ = true;
}
