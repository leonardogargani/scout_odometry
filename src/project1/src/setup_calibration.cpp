#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include <math.h>
#include <project1/robot_constants.h>

// the gear ratio is a value between 0 and 1
float computed_gear_ratio = 1;
float max_computed_gear_ratio = 0;
float min_computed_gear_ratio = 1;
float var_gear_ratio = 0;

// apparent baseline will be a value greater than 0.583 (real baseline)
float computed_apparent_baseline = 0.583;
float max_computed_apparent_baseline = 0;
float min_computed_apparent_baseline = 1000;
float var_apparent_baseline = 0;

// counters for iterations in the calibrations
unsigned int computation_iter_gear_ratio = 0;
unsigned int computation_iter_baseline = 0;

// compute and print to screen the apparent baseline, which is the baseline used in the approximation of
// the skid steering kinematics with the differential drive kinematics
void apparent_baseline_calibration(float odom_angular_vel, float vl_not_reduced, float vr_not_reduced)
{

	float vl = GEAR_RATIO * vl_not_reduced;
	float vr = GEAR_RATIO * vr_not_reduced;
	float current_apparent_baseline = (-vl + vr) / odom_angular_vel;

	// use only "good" data for the calibration process
	if (!(abs(odom_angular_vel) < 0.02) &&
		!isnan((computed_apparent_baseline * computation_iter_baseline + current_apparent_baseline) / (computation_iter_baseline + 1)))
	{
		// compute the empirical apparent baseline
		max_computed_apparent_baseline = std::max(current_apparent_baseline, max_computed_apparent_baseline);
		computed_apparent_baseline = (computed_apparent_baseline * computation_iter_baseline + current_apparent_baseline) / (computation_iter_baseline + 1);
		min_computed_apparent_baseline = std::min(current_apparent_baseline, min_computed_apparent_baseline);
		var_apparent_baseline = (var_apparent_baseline * computation_iter_baseline + pow((current_apparent_baseline - APPARENT_BASELINE), 2)) / (computation_iter_baseline + 1);
		computation_iter_baseline++;
	}

	ROS_INFO("\nComputed apparent baseline: %f\tmax: %f\tmin:  %f\tvar:  %f\n%d\n\n",
			 computed_apparent_baseline, max_computed_apparent_baseline, min_computed_apparent_baseline, var_apparent_baseline, computation_iter_baseline);
}

// compute and print to screen the gear ratio
void gear_ratio_calibration(float odom_vel, float vl_not_reduced, float vr_not_reduced)
{

	float linear_vel_not_reduced = (vl_not_reduced + vr_not_reduced) / 2;
	float current_gear_ratio = odom_vel / linear_vel_not_reduced;

	// use only "good" data for the calibration process
	if (!(abs(odom_vel) < 0.02) &&
		!isnan((computed_gear_ratio * computation_iter_gear_ratio + current_gear_ratio) / (computation_iter_gear_ratio + 1)))
	{
		// compute the empirical gear ratio
		max_computed_gear_ratio = std::max(current_gear_ratio, max_computed_gear_ratio);
		computed_gear_ratio = ((computed_gear_ratio * computation_iter_gear_ratio) + current_gear_ratio) / (computation_iter_gear_ratio + 1);
		min_computed_gear_ratio = std::min(current_gear_ratio, min_computed_gear_ratio);
		var_gear_ratio = (var_gear_ratio * computation_iter_gear_ratio + pow((current_gear_ratio - GEAR_RATIO), 2)) / (computation_iter_gear_ratio + 1);
		computation_iter_gear_ratio++;
	}

	ROS_INFO("\nComputed gear ratio: %f\tmax: %f\tmin:  %f\tvar:  %f\n%d\n\n",
			 computed_gear_ratio, max_computed_gear_ratio, min_computed_gear_ratio, var_gear_ratio, computation_iter_gear_ratio);
}

void callback(const robotics_hw1::MotorSpeed::ConstPtr &msg1,
			  const robotics_hw1::MotorSpeed::ConstPtr &msg2,
			  const robotics_hw1::MotorSpeed::ConstPtr &msg3,
			  const robotics_hw1::MotorSpeed::ConstPtr &msg4,
			  const nav_msgs::Odometry::ConstPtr &msg5)
{
	// motors on left and right wheels rotate in opposite directions when the robot goes straight
	float rpm_fl = -(msg1->rpm);
	float rpm_fr = msg2->rpm;
	float rpm_rl = -(msg3->rpm);
	float rpm_rr = msg4->rpm;

	// msg5 contains the linear velocity and angular velocity computed by the manufacturer odometry through encoders on the wheels
	float odom_vel = msg5->twist.twist.linear.x;
	float odom_angular_vel = msg5->twist.twist.angular.z;

	// velocities of robot wheels without considering the reduction gear
	float vl_not_reduced = ((rpm_fl + rpm_rl) / 2) * (1 / float(60)) * (2 * M_PI * WHEEL_RADIUS);
	float vr_not_reduced = ((rpm_fr + rpm_rr) / 2) * (1 / float(60)) * (2 * M_PI * WHEEL_RADIUS);

	apparent_baseline_calibration(odom_angular_vel, vl_not_reduced, vr_not_reduced);
	gear_ratio_calibration(odom_vel, vl_not_reduced, vr_not_reduced);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setup_calibration");

	ros::NodeHandle n;

	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fl(n, "motor_speed_fl", 1);
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fr(n, "motor_speed_fr", 1);
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rl(n, "motor_speed_rl", 1);
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rr(n, "motor_speed_rr", 1);

	message_filters::Subscriber<nav_msgs::Odometry> sub_scout_odom(n, "scout_odom", 1);

	typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed,
															robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry>
		MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr, sub_scout_odom);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

	ros::spin();

	return 0;
}
