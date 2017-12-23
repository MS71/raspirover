/*
 *
 */

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


std::string static_turtle_name;

int main(int argc, char **argv)
{
	ros::init(argc,argv, "raspirover_node");

	static tf2_ros::StaticTransformBroadcaster static_broadcaster;

	{
		/*
		 * <node pkg="tf" type="static_transform_publisher" name="tf_02" args="0 0 0 0 0 0 /base_footprint /base_link 100"/>
		 */
		geometry_msgs::TransformStamped static_transformStamped;
		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "base_footprint";
		static_transformStamped.child_frame_id = "base_link";
		static_transformStamped.transform.translation.x = 0;
		static_transformStamped.transform.translation.y = 0;
		static_transformStamped.transform.translation.z = 0;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, 0);
		static_transformStamped.transform.rotation.x = quat.x();
		static_transformStamped.transform.rotation.y = quat.y();
		static_transformStamped.transform.rotation.z = quat.z();
		static_transformStamped.transform.rotation.w = quat.w();
		static_broadcaster.sendTransform(static_transformStamped);
	}

	{
		/*
		 * <node pkg="tf" type="static_transform_publisher" name="tf_03" args="0.04 0.02 0.04 0 0 0 /base_link /imu_link 100"/>
		 */
		geometry_msgs::TransformStamped static_transformStamped;
		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "base_link";
		static_transformStamped.child_frame_id = "imu_link";
		static_transformStamped.transform.translation.x = 0.04;
		static_transformStamped.transform.translation.y = 0.02;
		static_transformStamped.transform.translation.z = 0.04;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, 0);
		static_transformStamped.transform.rotation.x = quat.x();
		static_transformStamped.transform.rotation.y = quat.y();
		static_transformStamped.transform.rotation.z = quat.z();
		static_transformStamped.transform.rotation.w = quat.w();
		static_broadcaster.sendTransform(static_transformStamped);
	}

	{
		/*
		 * <node pkg="tf" type="static_transform_publisher" name="tf_04" args="0.08 0.01 0.04 0 0 0 /base_link /sonar_link 100"/>
		 */
		geometry_msgs::TransformStamped static_transformStamped;
		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "base_link";
		static_transformStamped.child_frame_id = "sonar_link";
		static_transformStamped.transform.translation.x = 0.08;
		static_transformStamped.transform.translation.y = 0.01;
		static_transformStamped.transform.translation.z = 0.04;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, 0);
		static_transformStamped.transform.rotation.x = quat.x();
		static_transformStamped.transform.rotation.y = quat.y();
		static_transformStamped.transform.rotation.z = quat.z();
		static_transformStamped.transform.rotation.w = quat.w();
		static_broadcaster.sendTransform(static_transformStamped);
	}

	{
		/*
		 * <node pkg="tf" type="static_transform_publisher" name="tf_05" args="0.00 0.00 0.17 0 0 0 /base_link /laser_link 100"/>
		 */
		geometry_msgs::TransformStamped static_transformStamped;
		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "base_link";
		static_transformStamped.child_frame_id = "laser_link";
		static_transformStamped.transform.translation.x = 0.00;
		static_transformStamped.transform.translation.y = 0.00;
		static_transformStamped.transform.translation.z = 0.17;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, 0);
		static_transformStamped.transform.rotation.x = quat.x();
		static_transformStamped.transform.rotation.y = quat.y();
		static_transformStamped.transform.rotation.z = quat.z();
		static_transformStamped.transform.rotation.w = quat.w();
		static_broadcaster.sendTransform(static_transformStamped);
	}

	{
		/*
		 * <node pkg="tf" type="static_transform_publisher" name="tf_06" args="0.02 0.0 0.08 0 0 0 /base_link /camera_link 100"/>
		 */
		geometry_msgs::TransformStamped static_transformStamped;
		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "base_link";
		static_transformStamped.child_frame_id = "camera_link";
		static_transformStamped.transform.translation.x = 0.02;
		static_transformStamped.transform.translation.y = 0.00;
		static_transformStamped.transform.translation.z = 0.08;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, 0);
		static_transformStamped.transform.rotation.x = quat.x();
		static_transformStamped.transform.rotation.y = quat.y();
		static_transformStamped.transform.rotation.z = quat.z();
		static_transformStamped.transform.rotation.w = quat.w();
		static_broadcaster.sendTransform(static_transformStamped);
	}

	ros::spin();

	return 0;
};


#if 0
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

class RobotDriver
{
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to the "cmd_vel" topic to issue commands
	ros::Publisher cmd_vel_pub_;
	//! We will be listening to TF transforms as well
	tf::TransformListener listener_;

public:
	//! ROS node initialization
	RobotDriver(ros::NodeHandle &nh)
	{
		nh_ = nh;
		//set up the publisher for the cmd_vel topic
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	}

	//! Drive forward a specified distance based on odometry information
	bool driveForwardOdom(double distance)
	{
		//wait for the listener to get the first message
		listener_.waitForTransform("base_footprint", "odom",
				ros::Time(0), ros::Duration(5.0));

		//we will record transforms here
		tf::StampedTransform start_transform;
		tf::StampedTransform current_transform;

		//record the starting transform from the odometry to the base frame
		listener_.lookupTransform("base_footprint", "odom",
				ros::Time(0), start_transform);

		//we will be sending commands of type "twist"
		geometry_msgs::Twist base_cmd;
		//the command will be to go forward at 0.25 m/s
		base_cmd.linear.y = base_cmd.angular.z = 0;
		base_cmd.linear.x = 0.25;

		ros::Rate rate(10.0);
		bool done = false;
		printf("start %f ...\n",distance);
		while (!done && nh_.ok())
		{
			//send the drive command
			printf("cmd_vel_pub_.publish %f ...\n",base_cmd.linear.x);
			cmd_vel_pub_.publish(base_cmd);
			rate.sleep();
			//get the current transform
			try
			{
				listener_.lookupTransform("base_footprint", "odom",
						ros::Time(0), current_transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				break;
			}
			//see how far we've traveled
			tf::Transform relative_transform =
					start_transform.inverse() * current_transform;
			double dist_moved = relative_transform.getOrigin().length();

			printf("%f %f\n",dist_moved,distance);

			if(dist_moved > distance) done = true;
		}
		stop();

		printf("done ...\n");

		if (done) return true;
		return false;
	}

	void stop()
	{
		//stop
		geometry_msgs::Twist base_cmd;
		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
		cmd_vel_pub_.publish(base_cmd);
	}

	bool turnOdom(bool clockwise, double radians)
	{
		while(radians < 0) radians += 2*M_PI;
		while(radians > 2*M_PI) radians -= 2*M_PI;

		//wait for the listener to get the first message
		listener_.waitForTransform("base_footprint", "odom",
				ros::Time::now(), ros::Duration(5.0));

		//we will record transforms here
		tf::StampedTransform start_transform;
		tf::StampedTransform current_transform;

		//record the starting transform from the odometry to the base frame
		listener_.lookupTransform("base_footprint", "odom",
				ros::Time(0), start_transform);

		//we will be sending commands of type "twist"
		geometry_msgs::Twist base_cmd;
		//the command will be to turn at 0.75 rad/s
		base_cmd.linear.x = base_cmd.linear.y = 0.0;
		base_cmd.angular.z = 0.2;
		if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

		//the axis we want to be rotating by
		tf::Vector3 desired_turn_axis(0,0,1);
		if (!clockwise) desired_turn_axis = -desired_turn_axis;

		ros::Rate rate(10.0);
		bool done = false;

		printf("turnOdom(%d,%f) %f ...\n",clockwise,radians,base_cmd.angular.z);

		while (!done && nh_.ok())
		{
			//send the drive command
			cmd_vel_pub_.publish(base_cmd);

			rate.sleep();
			//get the current transform
			try
			{
				listener_.lookupTransform("base_footprint", "odom",
						ros::Time(0), current_transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				break;
			}
			tf::Transform relative_transform =
					start_transform.inverse() * current_transform;
			tf::Vector3 actual_turn_axis =
					relative_transform.getRotation().getAxis();
			double angle_turned = relative_transform.getRotation().getAngle();
			if ( fabs(angle_turned) < 1.0e-2) continue;

			if ( actual_turn_axis.dot( desired_turn_axis ) < 0 )
				angle_turned = 2 * M_PI - angle_turned;

			printf("turnOdom(%d,%f) %f %f ...\n",clockwise,radians,base_cmd.angular.z,angle_turned);

			if (angle_turned > radians) done = true;
		}
		stop();

		printf("turnOdom(%d,%f) %d ... done\n",clockwise,radians,done);

		if (done) return true;
		return false;
	}
};

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "raspirover_node");
	ros::NodeHandle nh;

	RobotDriver driver(nh);
	while(nh.ok())
	{
		//driver.driveForwardOdom(0.2);
		driver.turnOdom(true,M_PI);
		//driver.driveForwardOdom(0.2);
		driver.turnOdom(false,M_PI);
	}
}
#endif

/*
 * EOF
 */

