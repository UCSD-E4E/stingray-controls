#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

void os5000Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("message is: [%s]",msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("os5000_data",10000,os5000Callback);

    ros::spin();

    return 0;
}
