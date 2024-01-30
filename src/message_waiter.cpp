#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <vector>
#include <sstream>
#include <map>

std::vector<ros::Subscriber> subscribers; // Global vector to hold subscribers
std::map<std::string, bool> message_received_map;

void chatterCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic) {
    ROS_INFO("Message received on topic: %s", topic.c_str());
    message_received_map[topic] = true;

    // Find and remove the subscriber for this topic
    for (auto &sub : subscribers) {
        if (sub.getTopic() == topic || ("/" + sub.getTopic()) == topic) {
            sub.shutdown();
            break;
        }
    }
}

std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
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
        message_received_map[topic] = false;
        subscribers.push_back(nh.subscribe<topic_tools::ShapeShifter>(topic, 1, boost::bind(chatterCallback, _1, topic)));
    }

    ros::Rate loop_rate(1);
    bool all_received = false;
    while (ros::ok() && !all_received)
    {
        all_received = true;
        std::vector<std::string> pending_topics;

        for (const auto &topic_pair : message_received_map)
        {
            if (!topic_pair.second)
            {
                all_received = false;
                pending_topics.push_back(topic_pair.first);
            }
        }

        if (!all_received)
        {
            std::stringstream ss;
            ss << "Waiting for messages on topics: ";
            for (size_t i = 0; i < pending_topics.size(); ++i)
            {
                ss << pending_topics[i];
                if (i < pending_topics.size() - 1)
                {
                    ss << ", ";
                }
            }
            ROS_INFO_STREAM(ss.str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    if (all_received)
    {
        ROS_INFO("Messages received on all topics, running command: %s", command_to_run.c_str());
        int result = system(command_to_run.c_str());
        if (result != 0)
        {
            ROS_ERROR("Failed to execute command.");
        }
    }

    return 0;
}
