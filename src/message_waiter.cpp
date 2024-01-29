#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

bool message_received = false;
std::string command_to_run;
std::string topic_to_listen;

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: [%s]", msg->data.c_str());
    message_received = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "message_waiter");
    ros::NodeHandle nh;

    if (argc != 3) {
        ROS_ERROR("Usage: message_waiter <topic> <command>");
        return -1;
    }

    topic_to_listen = argv[1];
    command_to_run = argv[2];

    ros::Subscriber sub = nh.subscribe(topic_to_listen, 1000, chatterCallback);

    ros::Rate loop_rate(1);
    while (ros::ok() && !message_received) {
        ROS_INFO_STREAM("Waiting for message on topic " << topic_to_listen << "...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (message_received) {
        ROS_INFO("Message received, running command: %s", command_to_run.c_str());
        int result = system(command_to_run.c_str());
        if (result != 0) {
            ROS_ERROR("Failed to execute command.");
        }
    }

    return 0;
}
