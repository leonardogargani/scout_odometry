#include "ros/ros.h"
#include <math.h>
#include "robotics_hw1/MotorSpeed.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include <project1/robot_constants.h>

class Pub_sub
{

private:
	float rpm_fl;
	float rpm_fr;
	float rpm_rl;
	float rpm_rr;
	typedef message_filters::sync_policies ::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
															 robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed>
		MySyncPolicy;
	ros::NodeHandle n;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fl;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fr;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rl;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rr;
	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> sync;
	ros::Publisher pub;

public:
	Pub_sub()
	{

		sub_fl.subscribe(n, "motor_speed_fl", 1);
		sub_fr.subscribe(n, "motor_speed_fr", 1);
		sub_rl.subscribe(n, "motor_speed_rl", 1);
		sub_rr.subscribe(n, "motor_speed_rr", 1);

		sync.reset(new Sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr));
		sync->registerCallback(boost::bind(&Pub_sub::callback, this, _1, _2, _3, _4));

		pub = n.advertise<geometry_msgs::TwistStamped>("/velocity", 1);
	}

	void callback(const robotics_hw1::MotorSpeed::ConstPtr &msg1,
				  const robotics_hw1::MotorSpeed::ConstPtr &msg2,
				  const robotics_hw1::MotorSpeed::ConstPtr &msg3,
				  const robotics_hw1::MotorSpeed::ConstPtr &msg4)
	{
		rpm_fl = msg1->rpm;
		rpm_fr = msg2->rpm;
		rpm_rl = msg3->rpm;
		rpm_rr = msg4->rpm;
		ros::Time time = msg1->header.stamp;
		pub_velocity(time);
	}

	void pub_velocity(ros::Time time)
	{

		// motors on left and right wheels rotate in opposite directions when the robot goes straight
		rpm_fl = -rpm_fl;
		rpm_rl = -rpm_rl;

		// velocities of robot wheels without considering the reduction gear
		float vl = ((rpm_fl + rpm_rl) / 2) * (1 / float(60)) * (2 * M_PI * WHEEL_RADIUS) * GEAR_RATIO;
		float vr = ((rpm_fr + rpm_rr) / 2) * (1 / float(60)) * (2 * M_PI * WHEEL_RADIUS) * GEAR_RATIO;
		float linear_vel = (vl + vr) / 2;

		float angular_vel = (-vl + vr) / APPARENT_BASELINE;

		geometry_msgs::Twist tw;
		std_msgs::Header header;
		tw.linear.x = linear_vel;
		tw.angular.z = angular_vel;
		header.frame_id = "odom";
		header.stamp = time;

		geometry_msgs::TwistStamped message;
		message.header = header;
		message.twist = tw;
		pub.publish(message);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velocity_publisher");
	Pub_sub pub_sub;
	ros::spin();
	return 0;
}
