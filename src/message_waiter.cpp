#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <vector>
#include <sstream>

std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::map<std::string, bool> message_received_map;

void chatterCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic) {
    ROS_INFO("Message received on topic: %s", topic.c_str());
    message_received_map[topic] = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "message_waiter");
    ros::NodeHandle nh;

    if (argc != 3) {
        ROS_ERROR("Usage: message_waiter <topics> <command>");
        return -1;
    }

    std::vector<std::string> topics = split(argv[1], ',');
    std::string command_to_run = argv[2];

    for (const std::string& topic : topics) {
        message_received_map[topic] = false; // Initialize the map
        ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(topic, 1, boost::bind(chatterCallback, _1, topic));
    }

    ros::Rate loop_rate(1);
    bool all_received = false;
    while (ros::ok() && !all_received) {
        all_received = true;
        for (const auto& topic_pair : message_received_map) {
            if (!topic_pair.second) {
                all_received = false;
                ROS_INFO_STREAM("Waiting for message on topic " << topic_pair.first << "...");
                break; // Break as soon as we find one topic that hasn't received a message
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (all_received) {
        ROS_INFO("Messages received on all topics, running command: %s", command_to_run.c_str());
        int result = system(command_to_run.c_str());
        if (result != 0) {
            ROS_ERROR("Failed to execute command.");
        }
    }

    return 0;
}
