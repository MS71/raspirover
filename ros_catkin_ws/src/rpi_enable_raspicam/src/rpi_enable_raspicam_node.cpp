/*
 * enable the raspicam service
 */

#include <math.h>
#include <signal.h>

/*
 * ROS
 */
#include <ros/console.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_srvs/Empty.h>

/*
 * wiringPI
 */
#include <wiringPi.h>
#include <softPwm.h>

/*
 * GPIO pins
 */
int gpio_out_enable_laser = 23;		// enable/disable line laser

/*
 * control the line laser
 */
void laser_callback(const std_msgs::String& laser_cmd)
{
	char s[256];
	if( laser_cmd.data == "on" )
	{
		sprintf(s,"gpio -g write %d 1",gpio_out_enable_laser); system(s);
	}
	else
	{
		sprintf(s,"gpio -g write %d 0",gpio_out_enable_laser); system(s);
	}
}

/*
 * main
 */
int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "rpi_enable_raspicam" , ros::init_options::NoSigintHandler);

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle node;

	{
		char s[256];
		ROS_DEBUG("initializing GPIOs ...");
		sprintf(s,"gpio -g write %d 0",gpio_out_enable_laser); system(s);
	}

	wiringPiSetupSys();

	ros::ServiceClient client = node.serviceClient<std_srvs::Empty::Request>("/camera/start_capture");
	std_srvs::Empty start_capture_srv;
	while(client.call(start_capture_srv) == false)
	{
		ros::Duration(1.0).sleep();
	}

	ros::Rate loop_rate(1);
	while( node.ok() )
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/*
 * EOF
 */
