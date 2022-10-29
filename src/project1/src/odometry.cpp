#include "ros/ros.h"
#include "project1/CustomMessage.h"
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include "project1/integrationConfig.h"
#include <dynamic_reconfigure/server.h>
#include "project1/ResetPoseService.h"
#include "project1/SetPoseService.h"

struct Pose
{
	float x, y, yaw;
};

class Integrator
{

private:
	ros::NodeHandle n;
	ros::Publisher pub_odom;
	ros::Subscriber sub_velocity;
	ros::Publisher pub_custom;

	ros::ServiceServer resetter_service_server;
	ros::ServiceServer setter_service_server;

	Pose current_pose;
	ros::Time curr_time;
	ros::Time prev_time;

	unsigned int integration_method;

	dynamic_reconfigure::Server<project1::integrationConfig> server;
	dynamic_reconfigure::Server<project1::integrationConfig>::CallbackType f;

	bool reset_pose(project1::ResetPoseService::Request &req,
					project1::ResetPoseService::Response &res)
	{
		current_pose.x = 0;
		current_pose.y = 0;
		current_pose.yaw = 0;
		return true;
	}

	bool set_pose(project1::SetPoseService::Request &req,
				  project1::SetPoseService::Response &res)
	{
		current_pose.x = req.x;
		current_pose.y = req.y;
		current_pose.yaw = req.yaw;
		return true;
	}

	void odometry_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{

		// the first iteration (at t=0) there will be a delta_t = 0
		prev_time = (curr_time.isZero()) ? msg->header.stamp : curr_time;
		curr_time = msg->header.stamp;

		switch (integration_method)
		{
		case 0:
			euler_integration(msg->twist.linear.x, msg->twist.angular.z);
			break;
		case 1:
			runge_kutta_integration(msg->twist.linear.x, msg->twist.angular.z);
			break;
		default:
			ROS_ERROR("A non-implemented integration method has been set.");
		}

		pub_odometry(msg->twist);

		ROS_INFO("\nCurrent pose:\nx  %f m\ty  %f m\ttheta  %f\ntime  %f\n\n",
				 current_pose.x, current_pose.y, current_pose.yaw, curr_time.toSec());
	}

	void euler_integration(float linear_vel, float angular_vel)
	{
		float yaw = current_pose.yaw;
		float delta_t_sec = (float)(curr_time - prev_time).toSec();
		current_pose.yaw = yaw + angular_vel * delta_t_sec;
		current_pose.x = current_pose.x + linear_vel * delta_t_sec * cos(yaw);
		current_pose.y = current_pose.y + linear_vel * delta_t_sec * sin(yaw);
	}

	void runge_kutta_integration(float linear_vel, float angular_vel)
	{
		float yaw = current_pose.yaw;
		float delta_t_sec = (float)(curr_time - prev_time).toSec();
		current_pose.yaw = yaw + angular_vel * delta_t_sec;
		current_pose.x = current_pose.x + linear_vel * delta_t_sec * cos(yaw + (angular_vel * delta_t_sec / 2));
		current_pose.y = current_pose.y + linear_vel * delta_t_sec * sin(yaw + (angular_vel * delta_t_sec / 2));
	}

	void pub_odometry(geometry_msgs::Twist velocity)
	{
		nav_msgs::Odometry message;
		std_msgs::String integration_type;
		project1::CustomMessage custom_message;

		tf2::Quaternion quaternion;
		quaternion.setRPY(0, 0, current_pose.yaw);
		quaternion = quaternion.normalize();

		message.header.stamp = curr_time;
		message.header.frame_id = "odom";
		message.child_frame_id = "base_link";

		message.pose.pose.position.x = current_pose.x;
		message.pose.pose.position.y = current_pose.y;
		message.pose.pose.orientation.x = quaternion.x();
		message.pose.pose.orientation.y = quaternion.y();
		message.pose.pose.orientation.z = quaternion.z();
		message.pose.pose.orientation.w = quaternion.w();

		message.twist.twist.linear.x = velocity.linear.x;
		message.twist.twist.angular.z = velocity.angular.z;

		pub_odom.publish(message);

		custom_message.odom = message;

		switch (integration_method)
		{
		case 0:
			custom_message.method.data = "Euler";
			break;
		case 1:
			custom_message.method.data = "RK";
			break;
		default:
			ROS_ERROR("A non-implemented integration method has been set.");
		}

		pub_custom.publish(custom_message);
	}

	void parameters_callback(project1::integrationConfig &config, uint32_t level)
	{
		integration_method = config.integration;
	}

public:
	Integrator()
	{
		// dynamic reconfigure for the integration method
		f = boost::bind(&Integrator::parameters_callback, this, _1, _2);
		server.setCallback(f);

		pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1);
		sub_velocity = n.subscribe("/velocity", 10, &Integrator::odometry_callback, this);

		pub_custom = n.advertise<project1::CustomMessage>("/custom_topic", 1);

		resetter_service_server = n.advertiseService("reset_pose_service", &Integrator::reset_pose, this);
		setter_service_server = n.advertiseService("set_pose_service", &Integrator::set_pose, this);

		n.getParam("initial_pose_x", current_pose.x);
		n.getParam("initial_pose_y", current_pose.y);
		n.getParam("initial_pose_orientation", current_pose.yaw);

		curr_time = ros::Time(0, 0);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry");
	Integrator integrator;
	ros::spin();
	return 0;
}
