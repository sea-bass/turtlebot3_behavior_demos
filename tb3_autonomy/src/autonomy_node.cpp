#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "navigation_behaviors.h"
#include "vision_behaviors.h"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

// "Naive" implementation where there is a hard-coded sequence for each location.
// Here, there is a top-level fallback node that tries locations sequentially.
// As you can see, this does not automatically scale with number of locations.
static const char* xml_text_naive = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Fallback name="root">
            <Sequence name="search_location1">
                <GoToPose       name="go_to_location1"  loc="location1" />
                <LookForObject  name="look_in_location1"/>
            </Sequence>
            <Sequence name="search_location2">
                <GoToPose       name="go_to_location2" loc="location2"/>
                <LookForObject  name="look_in_location2"/>
            </Sequence>
            <Sequence name="search_location3">
                <GoToPose       name="go_to_location3" loc="location3"/>
                <LookForObject  name="look_in_location3"/>
            </Sequence>
            <Sequence name="search_location3">
                <GoToPose       name="go_to_location4" loc="location4"/>
                <LookForObject  name="look_in_location4"/>
            </Sequence>
        </Fallback>
     </BehaviorTree>
 </root>
 )";

// A better implementation which uses a queue of location names to iterate through 
// visiting locations regardless of number of locations.
static const char* xml_text_queue = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
    <Sequence name="main_loop">
        <SetLocations              name="set_locations" num_locs="{num_locs}"/>
        <RetryUntilSuccesful       num_attempts="{num_locs}">
            <Sequence   name="search_location">
                <GetLocationFromQueue   name="get_loc"      target_location="{target_location}"/>   
                <GoToPose               name="go_to_loc"    loc="{target_location}"/>
                <LookForObject          name="look_for_obj" />
            </Sequence>   
        </RetryUntilSuccesful>
    </Sequence>
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

    // Build a tree from XML and set it up for logging
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GoToPose>("GoToPose");
    factory.registerNodeType<SetLocations>("SetLocations");
    factory.registerNodeType<GetLocationFromQueue>("GetLocationFromQueue");
    factory.registerNodeType<LookForObject>("LookForObject");
    auto tree = factory.createTreeFromText(xml_text_queue);
    for (auto &node : tree.nodes) {
        if (auto vis_node = dynamic_cast<LookForObject*>(node.get())) {
            vis_node->init(nh);
        }
    }
    BT::PublisherZMQ publisher_zmq(tree);

    // Tick the tree until it reaches a terminal state
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        ros::Duration(0.5).sleep();
    }

    // Output final results
    std::string status_str;
    if (status == BT::NodeStatus::SUCCESS) {
        status_str = "SUCCESS";
    } else {
        status_str = "FAILURE";
    }
    ROS_INFO("Done with status %s!", status_str.c_str());

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
