#include "ros/ros.h"
#include "project1/ResetPoseService.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "reset_pose_client");

    if (argc != 1)
    {
        ROS_INFO("usage: reset_pose_service (with no parameters)");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project1::ResetPoseService>("reset_pose_service");
    project1::ResetPoseService srv;

    if (client.call(srv))
    {
        ROS_INFO("Pose has been reset to (0,0,0)");
    }
    else
    {
        ROS_ERROR("Failed to call service reset_pose_service");
        return 1;
    }

    return 0;
}
