#include "ros/ros.h"
#include "project1/SetPoseService.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "set_pose_client");

    if (argc != 4)
    {
        ROS_INFO("usage: set_pose_service x y yaw");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<project1::SetPoseService>("set_pose_service");
    project1::SetPoseService srv;

	// the arguments are read from the command line
	srv.request.x = atoll(argv[1]);
	srv.request.y = atoll(argv[2]);
	srv.request.yaw = atoll(argv[3]);

    if (client.call(srv))
    {
        ROS_INFO("Pose has been reset to (%f, %f, %f)", srv.request.x, srv.request.y, srv.request.yaw);
    }
    else
    {
        ROS_ERROR("Failed to call service set_pose_service");
        return 1;
    }

    return 0;
}
