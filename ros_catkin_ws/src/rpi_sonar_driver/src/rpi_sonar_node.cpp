/*
 * RPI HC04 sonar node
 */

#include <math.h>
#include <signal.h>

/*
 * ROS
 */
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

/*
 * wiringPI
 */
#include <wiringPi.h>
#include <softPwm.h>

/*
 * global data
 */

ros::Time sonar_last_time;
unsigned long t_sonar = 0;

ros::Publisher sonar_pub;
ros::Publisher scan_pub;

/*
 * GPIO pins
 */
int gpio_sonar = 4;		// HC04 sonar input

/*
 * sonar isr
 */
void sonarint(void)
{
	static int first = 1;
	static int _pin = 0;
	static ros::Time _t;
	ros::Time t = ros::Time::now();
	if( first == 0 )
	{
		int pin = digitalRead(gpio_sonar);
		if( pin != _pin )
		{
			_pin = pin;
			if( pin == 0 )
			{
				/*
				 * measure the HIGH pulse period
				 */
				t_sonar = ((4.0*t_sonar)+((t-_t).toSec()*1000000.0))/5.0;
			}
			_t = t;
		}
	}
	first = 0;
	return;
}

/*
 *
 */
void handleSonar()
{
	ros::Time current_time = ros::Time::now();
	double dt = (current_time - sonar_last_time).toSec();
	if(dt>=0.05)
	{
		sensor_msgs::Range range;
		range.header.frame_id = "sonar_link";
		range.radiation_type = sensor_msgs::Range::ULTRASOUND;
		range.min_range = 0.0;
		range.max_range = 0.5;
		range.field_of_view = 30.0/90.0 * 3.14/2.0;

		range.header.stamp = ros::Time::now();
		double r = 0.01 * t_sonar / 58.2;
		if( r > range.max_range )
		{
			// +Inf represents no detection within the fixed distance. (Object out of range)
			r = (1.0 / 0.0);
		}
		range.range = r;

		sonar_pub.publish(range);
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
		sprintf(s,"gpio export %d in",gpio_sonar); system(s);
	}

	wiringPiSetupSys();

	piHiPri(99);

	pwmSetMode(PWM_MODE_MS);

	pwmSetRange(1024);
	pwmSetClock(100);

	pinMode(gpio_sonar ,INPUT);
	wiringPiISR(gpio_sonar, INT_EDGE_BOTH, &sonarint);

	sonar_pub = node.advertise<sensor_msgs::Range>("sonar", 10);

	ros::Rate loop_rate(10);
	while( node.ok() )
	{
		handleSonar();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/*
 * EOF
 */
