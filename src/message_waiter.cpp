#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

bool message_received = false;

void chatterCallback(const topic_tools::ShapeShifter::ConstPtr &msg)
{
    ROS_INFO("Message received on topic");
    message_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_waiter");
    ros::NodeHandle nh;

    std::string topic_to_listen;
    std::string command_to_run;

    if (!nh.getParam("topic", topic_to_listen))
    {
        ROS_ERROR("Failed to get param 'topic'");
        return -1;
    }

    if (!nh.getParam("command", command_to_run))
    {
        ROS_ERROR("Failed to get param 'command'");
        return -1;
    }

    ros::Subscriber sub = nh.subscribe(topic_to_listen, 1, chatterCallback);

    ros::Rate loop_rate(1);
    while (ros::ok() && !message_received)
    {
        ROS_INFO_STREAM("Waiting for message on topic " << topic_to_listen << "...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (message_received)
    {
        ROS_INFO("Message received, running command: %s", command_to_run.c_str());
        int result = system(command_to_run.c_str());
        if (result != 0)
        {
            ROS_ERROR("Failed to execute command.");
        }
    }

    return 0;
}
