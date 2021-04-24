#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "navigation_behaviors.h"
#include "vision_behaviors.h"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

static const char* xml_text = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Fallback name="root">
            <Sequence name="search_location1">
                <GoToPosePort   name="go_to_location1"  loc="location1" />
                <LookForObject  name="look_in_location1"/>
            </Sequence>
            <Sequence name="search_location2">
                <GoToPosePort   name="go_to_location2" loc="location2"/>
                <LookForObject  name="look_in_location2"/>
            </Sequence>
            <Sequence name="search_location3">
                <GoToPosePort   name="go_to_location3" loc="location3"/>
                <LookForObject  name="look_in_location3"/>
            </Sequence>
            <Sequence name="search_location3">
                <GoToPosePort   name="go_to_location4" loc="location4"/>
                <LookForObject  name="look_in_location4"/>
            </Sequence>
        </Fallback>
     </BehaviorTree>
 </root>
 )";


void mainLoop(const ros::NodeHandle& nh) {
    // Read YAML file
    std::string yaml_file;
    ros::param::get("location_file", yaml_file);
    YAML::Node locations = YAML::LoadFile(yaml_file);
    std::vector<std::string> loc_names;
    for(YAML::const_iterator it=locations.begin(); it!=locations.end(); ++it) {
        loc_names.push_back(it->first.as<std::string>());
    }
    std::srand(100);
    std::random_shuffle(loc_names.begin(), loc_names.end());
    std::string loc = loc_names[0];

    // Build a tree from XML
    #define USE_XML
    #ifdef USE_XML
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<GoToPose>("GoToPose");
        factory.registerNodeType<GoToPosePort>("GoToPosePort");
        factory.registerNodeType<LookForObject>("LookForObject");
        auto tree = factory.createTreeFromText(xml_text);
        for (auto &node : tree.nodes) {
            if (auto vis_node = dynamic_cast<LookForObject*>(node.get())) {
                vis_node->init(nh);
            }
        }
    // Build a tree manually
    #else
        auto root = std::make_shared<BT::FallbackNode>("root");    
        BT::Tree tree;
        tree.nodes.push_back(root);
        for (std::string loc : loc_names) {
            std::vector<float> pose = locations[loc].as<std::vector<float>>();
            
            auto search = std::make_shared<BT::SequenceNode>("search_" + loc);
            root->addChild(search.get());
            tree.nodes.push_back(search);

            auto nav_node = std::make_shared<GoToPose>("go_to_" + loc, pose);
            search->addChild(nav_node.get());
            tree.nodes.push_back(nav_node);

            auto vis_node = std::make_shared<LookForObject>("look_in_" + loc);
            vis_node->init(nh);
            search->addChild(vis_node.get());
            tree.nodes.push_back(vis_node);
        }
    #endif

    // Set the tree up for logging
    BT::PublisherZMQ publisher_zmq(tree);

    // Tick the tree until it reaches a terminal state
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Done!");

}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "autonomy_node");
    ros::NodeHandle nh;

    // Run the main loop
    mainLoop(nh);
    return 0;
}
