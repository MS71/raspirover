/*
 * raspirover/generic RPI VL52LX servo based lidar node
 */

#include <math.h>
#include <signal.h>

/*
 * ROS
 */
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/*
 * wiringPI
 */
#include <wiringPi.h>
#include <softPwm.h>

/*
 * VL53L0X
 */
#include "VL53L0X.h"

/*
 * ROS parameter
 */

/*
 * GPIO pins
 */
int gpio_lidar_pwm = 12;		// Service PWM output
int gpio_lidar_gpio1 = 24;		// VL52L0X gpio1 pin
int gpio_lidar_xshut = 25;		// VL52L0X xshut pin

static enum {
	LIDAR_DIR_UP=0,
	LIDAR_DIR_DOWN
} lidar_pwm_dir = LIDAR_DIR_UP;
#define gpio_lidar_pwm_MAX 100
#define lidar_pwm_value_min (int)((0.50/10.0)*gpio_lidar_pwm_MAX)
#define lidar_pwm_value_max (int)((2.0/10.0)*gpio_lidar_pwm_MAX)
static int lidar_pwm_def_value = lidar_pwm_value_min + (lidar_pwm_value_max-lidar_pwm_value_min)/2;
static int lidar_pwm_value = lidar_pwm_def_value;
static int lidar_pwm_step = 1;
#define lidar_num_readings (1 + lidar_pwm_value_max - lidar_pwm_value_min)
static float lidar_data[lidar_num_readings] = {0.0};
ros::Time lidar_data_timestamps[lidar_num_readings];
static int lidar_send_topic = 0;
static int lidar_wait_countdown = 1;

bool param_updateparam = false;
double lidar_period = 0.05;
double lidar_angle_min = 0.0;
double lidar_angle_max = 0.0;

VL53L0X* sensor = NULL;
ros::Publisher scan_pub;

sensor_msgs::LaserScan lidar_scan;

/*
 * lidar handler function, control the servo and read the distance information
 */
ros::Time handle_lidar_time;
void handle_lidar()
{
	if( lidar_period == 0.0 )
		return;

	if( lidar_angle_min == 0.0 )
		return;

	if( lidar_angle_max == 0.0 )
		return;

	ros::Time time = ros::Time::now();
	double dt = (time - handle_lidar_time).toSec();
	if((dt >= lidar_period)&&(lidar_period != 0.0))
	{
		int i = lidar_pwm_value - lidar_pwm_value_min;
		double angle = lidar_angle_min + 1.0*i/lidar_num_readings * (lidar_angle_max-lidar_angle_min);

		/*
		 * read sensor value
		 */
		if( sensor != NULL )
		{
			lidar_data[i] = 0.001 * sensor->readRangeSingleMillimeters();
		}
		else
		{
			lidar_data[i] = 0.0;
		}
		lidar_data_timestamps[i] = time;

		if( lidar_send_topic == 1 )
		{
			lidar_send_topic = 0;
			sensor_msgs::LaserScan lidar_scan;
			lidar_scan.ranges.resize(lidar_num_readings);
			lidar_scan.intensities.resize(lidar_num_readings);
			lidar_scan.header.frame_id = "laser_link";
			if( lidar_pwm_dir == LIDAR_DIR_DOWN )
			{
				lidar_scan.header.stamp = lidar_data_timestamps[0];
				lidar_scan.angle_min = (2.0*3.14 * lidar_angle_min/360.0);
				lidar_scan.angle_max = (2.0*3.14 * lidar_angle_max/360.0);
				lidar_scan.angle_increment = ((lidar_scan.angle_max-lidar_scan.angle_min) / lidar_num_readings);
				for( int i=0;i<lidar_num_readings; i++ )
				{
					lidar_scan.ranges[i] = lidar_data[i];
					//lidar_scan.ranges[i]= 0.5;
					if( lidar_scan.ranges[i] > 2.0 )
					{
						lidar_scan.intensities[i] = 0.0;
					}
					else
					{
						lidar_scan.intensities[i] = 1.0;
					}
				}
				lidar_scan.time_increment = (lidar_data_timestamps[lidar_num_readings-1]-lidar_data_timestamps[0]).toSec() / (lidar_num_readings-1);
			}
			else
			{
				lidar_scan.header.stamp = lidar_data_timestamps[lidar_num_readings-1];
				lidar_scan.angle_min = (2.0*3.14 * lidar_angle_max/360.0);
				lidar_scan.angle_max = (2.0*3.14 * lidar_angle_min/360.0);
				lidar_scan.angle_increment = ((lidar_scan.angle_max-lidar_scan.angle_min) / lidar_num_readings);
				for( int i=0;i<lidar_num_readings; i++ )
				{
					lidar_scan.ranges[i] = lidar_data[lidar_num_readings-1-i];
					//lidar_scan.ranges[i]= 0.5;
					if( lidar_scan.ranges[i] > 2.0 )
					{
						lidar_scan.intensities[i] = 0.0;
					}
					else
					{
						lidar_scan.intensities[i] = 1.0;
					}
				}
				lidar_scan.time_increment = (lidar_data_timestamps[0]-lidar_data_timestamps[lidar_num_readings-1]).toSec() / (lidar_num_readings-1);
			}
			lidar_scan.range_min = 0.0;
			lidar_scan.range_max = 2.0;

			if( lidar_wait_countdown > 0 )
			{
				lidar_wait_countdown--;
			}

			if( lidar_wait_countdown == 0 )
			{
				scan_pub.publish(lidar_scan);
			}

		}

		if( lidar_pwm_dir == LIDAR_DIR_UP )
		{
			lidar_pwm_value += lidar_pwm_step;
			if( lidar_pwm_value >= lidar_pwm_value_max )
			{
				lidar_pwm_value = lidar_pwm_value_max;
				lidar_pwm_dir = LIDAR_DIR_DOWN;
				lidar_send_topic = 1;
			}
		}
		else
		{
			lidar_pwm_value -= lidar_pwm_step;
			if( lidar_pwm_value <= lidar_pwm_value_min )
			{
				lidar_pwm_value = lidar_pwm_value_min;
				lidar_pwm_dir = LIDAR_DIR_UP;
				lidar_send_topic = 1;
			}
		}

		/*
		 * next servo angle ...
		 */
		if( gpio_lidar_pwm != 0 )
		{
			softPwmWrite (gpio_lidar_pwm, lidar_pwm_value) ;
		}

		handle_lidar_time = time;
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
	ros::init(argc, argv, "rpi_servo_vl53lx_lidar_driver_node" , ros::init_options::NoSigintHandler);

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle node("rpi_servo_vl53lx_lidar_driver_node");

	/*
	 * read parameter
	 */
	node.getParam("updateparam", param_updateparam);
	node.getParam("lidar_period", lidar_period);
	node.getParam("lidar_angle_min", lidar_angle_min);
	node.getParam("lidar_angle_max", lidar_angle_max);

	node.getParam("gpio_out_pwm", gpio_lidar_pwm);
	node.getParam("gpio_in_gpio1", gpio_lidar_gpio1);
	node.getParam("gpio_out_xshut", gpio_lidar_xshut);

	{
		char s[256];
		ROS_DEBUG("initializing GPIOs ...");
		sprintf(s,"gpio export %d out",gpio_lidar_pwm); system(s);
		sprintf(s,"gpio export %d in",gpio_lidar_gpio1); system(s);
		sprintf(s,"gpio export %d out",gpio_lidar_xshut); system(s);
	}

	wiringPiSetupSys();

	piHiPri(99);

	pwmSetMode(PWM_MODE_MS);

	pwmSetRange(1024);
	pwmSetClock(100);

	sensor = NULL;
	if( gpio_lidar_xshut != 0 )
	{
		/*
		 * enable VL53L0X
		 */
		pinMode(gpio_lidar_xshut,OUTPUT);
		digitalWrite (gpio_lidar_xshut, HIGH);

		sensor = new VL53L0X(gpio_lidar_xshut);
		sensor->init();
		sensor->setTimeout(200);

		// Lower the return signal rate limit (default is 0.25 MCPS)
		sensor->setSignalRateLimit(0.25);
		// Increase laser pulse periods (defaults are 14 and 10 PCLKs)
		sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
		sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

		// Reduce timing budget to 20 ms (default is about 33 ms)
		sensor->setMeasurementTimingBudget(33000);
	}

	if( gpio_lidar_pwm != 0 )
	{
		pinMode(gpio_lidar_pwm,PWM_OUTPUT);
		softPwmCreate(gpio_lidar_pwm, lidar_pwm_value, gpio_lidar_pwm_MAX);
	}

	scan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 1.0/(1.0 * lidar_num_readings * 0.05));

	ros::Rate loop_rate(100);
	while( node.ok() )
	{
		if( param_updateparam==true )
		{
			node.getParam("updateparam", param_updateparam);

			node.getParam("lidar_period", lidar_period);
			node.getParam("lidar_angle_min", lidar_angle_min);
			node.getParam("lidar_angle_max", lidar_angle_max);
			//printf("lidar: %d %f %f\n",param_updateparam,lidar_angle_min,lidar_angle_max);
		}

		handle_lidar();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/*
 * EOF
 */
